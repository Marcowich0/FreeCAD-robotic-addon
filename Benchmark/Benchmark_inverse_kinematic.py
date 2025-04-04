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
current_dir = os.path.dirname(os.path.dirname(os.path.abspath(inspect.getsourcefile(lambda:0))))
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
    tolerance = 1e-6
    damping = 0.1
    orientation_weight = 1.0
    alpha = 0.01  # null-space weight for smoothness

    # If target_dir is a valid direction vector
    target_active = abs(np.linalg.norm(target_dir) - 1) < 1e-4

    # Store the initial configuration for null-space projection
    q_init = q.copy()
    
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
        J = J_full if target_active else J_full[:3, :]

        # Damped least squares
        J_T = J.T
        JJT = J @ J_T
        m = J.shape[0]
        damping_matrix = (damping**2) * np.eye(m)
        J_pseudo = J_T @ np.linalg.inv(JJT + damping_matrix)

        # Null-space projection
        I = np.eye(len(q))
        N = I - J_pseudo @ J

        # Smoothness optimization: pull configuration toward initial configuration
        z = q_init - q

        # Full update with null-space term
        delta_theta = J_pseudo @ delta_x + alpha * (N @ z)
        q = q + delta_theta.flatten()

    return False  # No solution within max_iterations

def build_target_T(target_pos, target_dir):
    """Construct a homogeneous transformation with translation = target_pos and
    z-axis = target_dir (with arbitrary x and y axes)."""
    z = target_dir / np.linalg.norm(target_dir)
    # Choose an arbitrary vector that is not parallel to z.
    if np.allclose(z, np.array([0, 0, 1])):
        x_temp = np.array([1, 0, 0])
    else:
        x_temp = np.array([0, 0, 1])
    x = np.cross(x_temp, z)
    if np.linalg.norm(x) < 1e-6:
        x = np.array([1, 0, 0])
    else:
        x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.column_stack((x, y, z))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = target_pos
    return T

def solve_ik_roboticstoolbox(q_initial, target_pos, target_dir, dh):
    """Solve IK using Peter Corke's Robotics Toolbox."""
    from roboticstoolbox import DHRobot, RevoluteDH
    # Build a robot model from DH parameters.
    robot = DHRobot([RevoluteDH(d=link[1], a=link[2], alpha=link[3]) for link in dh])
    T_target = build_target_T(target_pos, target_dir)
    # Use the Levenberg–Marquardt IK solver.
    sol = robot.ikine_LM(T_target, q0=q_initial, tol=1e-6)
    if sol.success:
        return sol.q
    else:
        return False

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
    Runs IK once with all three solvers, returning their times in seconds as
    (time_cpp, time_old, time_rtb).
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

        # Time the robotics toolbox solver
        start = time.perf_counter()
        solve_ik_roboticstoolbox(q_initial, target_pos, target_dir, dh)
        time_rtb = time.perf_counter() - start

        return (time_cpp, time_old, time_rtb)
    except Exception as e:
        print("Error in iteration:", e)
        return None

def benchmark_parallel(max_dof=32, runs_per_dof=300):
    """
    Benchmark all three IK solvers in parallel, testing only exponential
    DOF values (powers of two) up to 'max_dof'.
    """
    # Generate DOF values: 2, 4, 8, ..., up to max_dof
    dof_values = []
    power = 3
    while (1 << power) <= max_dof:
        dof_values.append(1 << power)
        power += 1
    
    # Dictionary: dof -> ( [cpp_times], [old_times], [rtb_times] )
    results = {}
    
    with mp.Pool(processes=mp.cpu_count()) as pool:
        for dof in dof_values:
            print(f"Testing {dof} DOF...")
            
            cpp_times = []
            old_times = []
            rtb_times = []
            chunk_size = max(50, runs_per_dof // 100)
            
            for outcome in pool.imap_unordered(
                partial(run_single_iteration),
                [dof]*runs_per_dof,
                chunksize=chunk_size
            ):
                if outcome is not None:
                    t_cpp, t_old, t_rtb = outcome
                    cpp_times.append(t_cpp)
                    old_times.append(t_old)
                    rtb_times.append(t_rtb)
            
            if cpp_times:
                results[dof] = (cpp_times, old_times, rtb_times)
            else:
                results[dof] = None

    return results

def plot_performance_line(results):
    """
    Creates a line plot of median times vs. DOF for all three solvers, with points
    connected, and error bars marking the 5th and 95th percentiles. Both axes are in log scale.
    """
    plt.figure(figsize=(14, 8))
    
    # Collect DOFs in sorted order
    sorted_dofs = sorted(d for d in results if results[d] is not None)
    
    dofs = []
    median_cpp, p5_cpp, p95_cpp = [], [], []
    median_old, p5_old, p95_old = [], [], []
    median_rtb, p5_rtb, p95_rtb = [], [], []
    
    for dof in sorted_dofs:
        cpp_times, old_times, rtb_times = results[dof]
        cpp_ms = np.array(cpp_times) * 1000  # ms
        old_ms = np.array(old_times) * 1000
        rtb_ms = np.array(rtb_times) * 1000
        
        dofs.append(dof)
        median_cpp.append(np.median(cpp_ms))
        p5_cpp.append(np.percentile(cpp_ms, 5))
        p95_cpp.append(np.percentile(cpp_ms, 95))
        median_old.append(np.median(old_ms))
        p5_old.append(np.percentile(old_ms, 5))
        p95_old.append(np.percentile(old_ms, 95))
        median_rtb.append(np.median(rtb_ms))
        p5_rtb.append(np.percentile(rtb_ms, 5))
        p95_rtb.append(np.percentile(rtb_ms, 95))
    
    dofs = np.array(dofs)
    median_cpp = np.array(median_cpp)
    p5_cpp = np.array(p5_cpp)
    p95_cpp = np.array(p95_cpp)
    median_old = np.array(median_old)
    p5_old = np.array(p5_old)
    p95_old = np.array(p95_old)
    median_rtb = np.array(median_rtb)
    p5_rtb = np.array(p5_rtb)
    p95_rtb = np.array(p95_rtb)
    
    cpp_lower_err = median_cpp - p5_cpp
    cpp_upper_err = p95_cpp - median_cpp
    old_lower_err = median_old - p5_old
    old_upper_err = p95_old - median_old
    rtb_lower_err = median_rtb - p5_rtb
    rtb_upper_err = p95_rtb - median_rtb

    plt.errorbar(dofs, median_cpp, yerr=[cpp_lower_err, cpp_upper_err],
                 fmt='-o', capsize=4, label='C++', color='blue', alpha=0.8)
    plt.errorbar(dofs, median_old, yerr=[old_lower_err, old_upper_err],
                 fmt='-s', capsize=4, label='Python', color='red', alpha=0.8)
    plt.errorbar(dofs, median_rtb, yerr=[rtb_lower_err, rtb_upper_err],
                 fmt='-^', capsize=4, label='Robotics Toolbox', color='green', alpha=0.8)
    
    plt.xscale('log', base=2)
    plt.yscale('log')
    ax = plt.gca()
    ax.yaxis.grid(True, which='major', linestyle='-')
    ax.yaxis.grid(True, which='minor', linestyle=':')
    ax.yaxis.set_minor_locator(AutoMinorLocator())
    plt.xticks(dofs, [str(d) for d in dofs])
    plt.xlim(dofs[0] / 1.5, dofs[-1] * 1.5)
    plt.xlabel('Degrees of Freedom [log2 Scale]', fontsize=12)
    plt.ylabel('Computation Time (ms) [Median with 5th-95th Percentile Range; log Scale]', fontsize=12)
    plt.title('IK Solver Performance: C++ vs. Python vs. Robotics Toolbox', fontsize=14)
    plt.legend()
    plt.tight_layout()
    plt.show()

import csv

def save_results_to_csv(results, filename="Benchmark/Benchmark_inverse_kinematic_results.csv"):
    """
    Saves the benchmark summary (median, 5th & 95th-percentile times) for all three solvers to a CSV file.
    """
    sorted_dofs = sorted(d for d in results if results[d] is not None)
    
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["DOF", "Samples", 
                         "CPP_Median_ms", "CPP_5th_ms", "CPP_95th_ms",
                         "PY_Median_ms", "PY_5th_ms", "PY_95th_ms",
                         "RTB_Median_ms", "RTB_5th_ms", "RTB_95th_ms"])
        
        for dof in sorted_dofs:
            cpp_times, py_times, rtb_times = results[dof]
            cpp_ms = np.array(cpp_times) * 1000
            py_ms = np.array(py_times) * 1000
            rtb_ms = np.array(rtb_times) * 1000
            
            writer.writerow([
                dof,
                len(cpp_times),
                np.median(cpp_ms),
                np.percentile(cpp_ms, 5),
                np.percentile(cpp_ms, 95),
                np.median(py_ms),
                np.percentile(py_ms, 5),
                np.percentile(py_ms, 95),
                np.median(rtb_ms),
                np.percentile(rtb_ms, 5),
                np.percentile(rtb_ms, 95)
            ])

if __name__ == "__main__":
    # Ensure the Robotics Toolbox is available.
    try:
        from roboticstoolbox import DHRobot, RevoluteDH
    except ImportError:
        print("Please install 'roboticstoolbox-python' to benchmark the Robotics Toolbox IK solver.")
        sys.exit(1)
    
    max_dof = 2**6
    runs_per_dof = 300
    
    benchmark_results = benchmark_parallel(max_dof, runs_per_dof)
    
    print("\nBenchmark Summary:")
    header = ("DOF", "Samples", 
              "C++ Median (ms)", "C++ 5th (ms)", "C++ 95th (ms)",
              "Python Median (ms)", "Python 5th (ms)", "Python 95th (ms)",
              "RTB Median (ms)", "RTB 5th (ms)", "RTB 95th (ms)")
    print(" | ".join(f"{h:<18}" for h in header))
    
    for dof in sorted(benchmark_results.keys()):
        if benchmark_results[dof] is not None:
            cpp_times, py_times, rtb_times = benchmark_results[dof]
            cpp_arr = np.array(cpp_times) * 1000
            py_arr = np.array(py_times) * 1000
            rtb_arr = np.array(rtb_times) * 1000
            print(" | ".join([
                f"{dof:<18}",
                f"{len(cpp_times):<18}",
                f"{np.median(cpp_arr):<18.3f}",
                f"{np.percentile(cpp_arr, 5):<18.3f}",
                f"{np.percentile(cpp_arr, 95):<18.3f}",
                f"{np.median(py_arr):<18.3f}",
                f"{np.percentile(py_arr, 5):<18.3f}",
                f"{np.percentile(py_arr, 95):<18.3f}",
                f"{np.median(rtb_arr):<18.3f}",
                f"{np.percentile(rtb_arr, 5):<18.3f}",
                f"{np.percentile(rtb_arr, 95):<18.3f}"
            ]))
    
    save_results_to_csv(benchmark_results)
    plot_performance_line(benchmark_results)
