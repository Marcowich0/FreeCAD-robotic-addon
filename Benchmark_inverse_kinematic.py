import sys
import os
import inspect
import time
import numpy as np
import matplotlib.pyplot as plt
import multiprocessing as mp
from functools import partial
from matplotlib.ticker import AutoMinorLocator

# Path configuration
current_dir = os.path.dirname(os.path.abspath(inspect.getsourcefile(lambda:0)))
module_path = os.path.join(current_dir, "robot_dynamics_module", "build", "Release")
sys.path.append(module_path)

import inverse_kinematics_cpp

def getDHTransformations(q, dh):
    last = None
    trans = [np.eye(4)]
    for dh_parameters, theta in zip(dh, q):
        _, d, a, alpha = dh_parameters
        dh_trans = np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),               np.cos(alpha),               d],
            [0,              0,                           0,                           1]
        ])
        last = dh_trans if last is None else last @ dh_trans
        trans.append(last)
    return trans

def getJacobian(q, dh):
    T_arr = getDHTransformations(q, dh)
    On = T_arr[-1][0:3, 3]
    J_cols = []
    for i in range(1, len(T_arr)):
        Zi = T_arr[i-1][0:3, 2]
        Oi = T_arr[i-1][0:3, 3]
        Jv = np.cross(Zi, (On - Oi))
        Jw = Zi
        J_cols.append([*Jv, *Jw])
    return np.array(J_cols).T  # shape (6, n)

def solve_ik_old(q, target_pos, target_dir, dh):
    max_iterations = 300
    tolerance = 0.1
    damping = 0.1
    orientation_weight = 1.0

    # If target_dir is a valid direction vector
    target_active = abs(np.linalg.norm(target_dir) - 1) < 1e-4
    
    for _ in range(max_iterations):
        T_arr = getDHTransformations(q, dh)
        current_pos = (T_arr[-1] @ np.array([0, 0, 0, 1]))[:3]

        delta_x_position = target_pos - current_pos
        if target_active:
            current_dir = T_arr[-1][:3, 2]
            orientation_error_vec = np.cross(current_dir, target_dir)
            delta_x_orientation = orientation_weight * orientation_error_vec
            delta_x = np.concatenate([delta_x_position, delta_x_orientation])
        else:
            delta_x = delta_x_position

        total_error = np.linalg.norm(delta_x)
        if total_error < tolerance:
            return q

        # Jacobian
        J_full = getJacobian(q, dh)
        # position-only or position+orientation
        J = J_full if target_active else J_full[:3, :]

        # Damped least squares
        J_T = J.T
        JJT = J @ J_T
        m = J.shape[0]
        damping_matrix = (damping**2) * np.eye(m)
        J_pseudo = J_T @ np.linalg.inv(JJT + damping_matrix)

        # Update q
        delta_theta = J_pseudo @ delta_x
        q = q + delta_theta.flatten()

    return False  # No solution within max_iterations

def generate_random_dh(n_dof):
    """Generate random DH parameters for each joint."""
    return [
        [0.0,
         np.random.uniform(0.05, 0.5),   # d
         np.random.uniform(0.1, 1.0),    # a
         np.random.uniform(-np.pi, np.pi)]  # alpha
        for _ in range(n_dof)
    ]

def compute_random_target(dh_params):
    """Generate a random valid target using forward kinematics."""
    q_true = np.random.uniform(-np.pi, np.pi, len(dh_params))
    T = np.eye(4)
    for i, (theta, d, a, alpha) in enumerate(dh_params):
        theta_total = theta + q_true[i]
        ct, st = np.cos(theta_total), np.sin(theta_total)
        ca, sa = np.cos(alpha), np.sin(alpha)
        T = T @ np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,      sa,     ca,    d],
            [0,       0,      0,    1]
        ])
    return q_true, T[:3, 3], T[:3, 0]

def run_single_iteration(dof):
    """
    Runs IK once with both the new C++ solver and the old solver,
    returning their times in seconds as (time_cpp, time_old).
    """
    try:
        dh = generate_random_dh(dof)
        q_true, target_pos, target_dir = compute_random_target(dh)
        q_initial = np.random.uniform(-np.pi, np.pi, dof)
        
        # Time the new (C++) solver
        start = time.perf_counter()
        inverse_kinematics_cpp.solveIK(q_initial, target_pos, target_dir, dh)
        time_cpp = time.perf_counter() - start

        # Time the old (Python) solver
        start = time.perf_counter()
        solve_ik_old(q_initial, target_pos, target_dir, dh)
        time_old = time.perf_counter() - start

        return (time_cpp, time_old)
    except:
        return None

def benchmark_parallel(max_dof=32, runs_per_dof=300):
    """
    Benchmark both IK solvers in parallel, testing only exponential
    DOF values (powers of two) up to 'max_dof'.
    """
    # Generate DOF values: 2, 4, 8, ..., up to max_dof
    dof_values = []
    power = 3
    while (1 << power) <= max_dof:
        dof_values.append(1 << power)
        power += 1
    
    # Dictionary: dof -> ( [cpp_times], [old_times] )
    results = {}
    
    with mp.Pool(processes=mp.cpu_count()) as pool:
        for dof in dof_values:
            print(f"Testing {dof} DOF...")
            
            cpp_times = []
            old_times = []
            chunk_size = max(50, runs_per_dof // 100)
            
            for outcome in pool.imap_unordered(
                partial(run_single_iteration),
                [dof]*runs_per_dof,
                chunksize=chunk_size
            ):
                if outcome is not None:
                    t_cpp, t_old = outcome
                    cpp_times.append(t_cpp)
                    old_times.append(t_old)
            
            if cpp_times:
                results[dof] = (cpp_times, old_times)
            else:
                results[dof] = None

    return results

def plot_performance_line(results):
    """
    Creates a line plot of median times vs. DOF, with points connected,
    and error bars marking the 5th and 95th percentiles for each solver.
    Both x-axis and y-axis are in log scale (base-2 for x, base-10 for y).
    """
    plt.figure(figsize=(14, 8))
    
    # Collect DOFs in sorted order
    sorted_dofs = sorted(d for d in results if results[d] is not None)
    
    # Prepare arrays for each solver
    dofs = []
    median_cpp = []
    p5_cpp = []
    p95_cpp = []
    median_old = []
    p5_old = []
    p95_old = []
    
    for dof in sorted_dofs:
        cpp_times, old_times = results[dof]
        cpp_times_ms = np.array(cpp_times) * 1000  # Convert to ms
        old_times_ms = np.array(old_times) * 1000
        
        dofs.append(dof)
        median_cpp.append(np.median(cpp_times_ms))
        p5_cpp.append(np.percentile(cpp_times_ms, 5))
        p95_cpp.append(np.percentile(cpp_times_ms, 95))
        median_old.append(np.median(old_times_ms))
        p5_old.append(np.percentile(old_times_ms, 5))
        p95_old.append(np.percentile(old_times_ms, 95))
    
    # Convert to NumPy arrays
    dofs = np.array(dofs)
    median_cpp = np.array(median_cpp)
    p5_cpp = np.array(p5_cpp)
    p95_cpp = np.array(p95_cpp)
    median_old = np.array(median_old)
    p5_old = np.array(p5_old)
    p95_old = np.array(p95_old)
    
    # Calculate error ranges: lower (median - 5th) and upper (95th - median)
    cpp_lower_err = median_cpp - p5_cpp
    cpp_upper_err = p95_cpp - median_cpp
    old_lower_err = median_old - p5_old
    old_upper_err = p95_old - median_old

    # Plot line + points + error bars for C++ solver
    plt.errorbar(
        dofs,
        median_cpp,
        yerr=[cpp_lower_err, cpp_upper_err],
        fmt='-o',
        capsize=4,
        label='C++',
        color='blue',
        alpha=0.8
    )
    
    # Plot line + points + error bars for Old solver
    plt.errorbar(
        dofs,
        median_old,
        yerr=[old_lower_err, old_upper_err],
        fmt='-s',
        capsize=4,
        label='Python',
        color='red',
        alpha=0.8
    )
    
    # Apply log scale to both axes
    plt.xscale('log', base=2)  # log base-2 for x
    plt.yscale('log')          # log base-10 for y (default)

    ax = plt.gca()
    ax.yaxis.grid(True, which='major', linestyle='-')
    ax.yaxis.grid(True, which='minor', linestyle=':')
    ax.yaxis.set_minor_locator(AutoMinorLocator())

    # Explicitly set x-ticks to the actual DOFs (2, 4, 8, ...)
    plt.xticks(dofs, [str(d) for d in dofs])
    
    # A little space on either side if you like
    plt.xlim(dofs[0] / 1.5, dofs[-1] * 1.5)
    
    plt.xlabel('Degrees of Freedom [log2 Scale]', fontsize=12)
    plt.ylabel('Computation Time (ms) [Median with 5th-95th Percentile Range; log Scale]', fontsize=12)
    plt.title('IK Solver Performance: C++ vs. Python Implementation', fontsize=14)
    plt.legend()
    plt.tight_layout()
    plt.savefig('ik_solver_comparison_lineplot.png', dpi=300)
    plt.show()

import csv

def save_results_to_csv(results, filename="Benchmark_inverse_kinematic_results.csv"):
    """
    Saves the benchmark summary (median, 5th & 95th-percentile times) to a CSV file.
    """
    sorted_dofs = sorted(d for d in results if results[d] is not None)
    
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["DOF", "Samples", 
                        "CPP_Median_ms", "CPP_5th_ms", "CPP_95th_ms",
                        "PY_Median_ms", "PY_5th_ms", "PY_95th_ms"])
        
        for dof in sorted_dofs:
            cpp_times, py_times = results[dof]
            cpp_ms = np.array(cpp_times) * 1000
            py_ms = np.array(py_times) * 1000
            
            writer.writerow([
                dof,
                len(cpp_times),
                np.median(cpp_ms),
                np.percentile(cpp_ms, 5),
                np.percentile(cpp_ms, 95),
                np.median(py_ms),
                np.percentile(py_ms, 5),
                np.percentile(py_ms, 95)
            ])

if __name__ == "__main__":
    max_dof = 2**12
    runs_per_dof = 100
    
    benchmark_results = benchmark_parallel(max_dof, runs_per_dof)
    
    print("\nBenchmark Summary:")
    print(f"{'DOF':<5} | {'Samples':<8} "
          f"| {'C++ Median (ms)':<15} | {'C++ 5th (ms)':<14} | {'C++ 95th (ms)':<14} "
          f"| {'Old Median (ms)':<15} | {'Old 5th (ms)':<14} | {'Old 95th (ms)':<14}")
    
    for dof in sorted(benchmark_results.keys()):
        if benchmark_results[dof] is not None:
            cpp_times, old_times = benchmark_results[dof]
            cpp_arr = np.array(cpp_times) * 1000
            old_arr = np.array(old_times) * 1000
            
            print(f"{dof:<5} | {len(cpp_times):<8} "
                  f"| {np.median(cpp_arr):<15.3f} | {np.percentile(cpp_arr, 5):<14.3f} | {np.percentile(cpp_arr, 95):<14.3f} "
                  f"| {np.median(old_arr):<15.3f} | {np.percentile(old_arr, 5):<14.3f} | {np.percentile(old_arr, 95):<14.3f}")
    
    save_results_to_csv(benchmark_results)
    plot_performance_line(benchmark_results)
