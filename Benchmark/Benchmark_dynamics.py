import sys
import os
import inspect
import time
import numpy as np
import matplotlib.pyplot as plt
import multiprocessing as mp
from functools import partial
from matplotlib.ticker import AutoMinorLocator
import csv

# Path configuration
current_dir = os.path.dirname(os.path.dirname(os.path.abspath(inspect.getsourcefile(lambda:0))))
module_path = os.path.join(current_dir, "robot_dynamics_module", "build", "Release")
sys.path.append(module_path)

import compute_torque  # C++ module for torque computation

def partial_derivative(f, q, i, h=1e-6):
    q_plus = q.copy()
    q_minus = q.copy()
    q_plus[i] += h
    q_minus[i] -= h
    return (f(q_plus) - f(q_minus)) / (2 * h)    

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

def getJacobianCenter(q, dh, centerofmass):
    T_arr = getDHTransformations(q, dh)
    Jac_arr = []
    for i in range(1, len(T_arr)):
        Jac = []
        On = (T_arr[i] @ np.array([*centerofmass[i], 1]))[:3]
        for j in range(1, len(T_arr)):
            if j <= i:
                Zi = T_arr[j-1][0:3, 2]
                Oi = T_arr[j-1][0:3, 3]
                Jv = np.cross(Zi, (On - Oi))
                Jw = Zi
            else:
                Jv = np.zeros(3)
                Jw = np.zeros(3)
            Jac.append([*Jv, *Jw])
        Jac_arr.append(np.array(Jac).T.astype(float))
    return Jac_arr

def computeJointTorques(q, q_dot, q_ddot, M, InertiaMatrices, CenterOfMass, DHparameters):
    gravity = np.array([0, 0, -9.81])
    def D(q):
        # Note: The base (dummy) link is skipped for dynamics.
        Jac_vci, Jac_wi = map(list, zip(*[np.split(jac, [3], axis=0) 
                                          for jac in getJacobianCenter(q, DHparameters, CenterOfMass)[1:]]))
        I_global = [R[:3, :3] @ I @ R[:3, :3].T 
                    for R, I in zip(getDHTransformations(q, DHparameters)[1:], InertiaMatrices[1:])]
        D_out = sum([
            m_i * Jac_v.T @ Jac_v + Jac_w.T @ J_i @ Jac_w 
            for m_i, Jac_v, Jac_w, J_i in zip(M, Jac_vci, Jac_wi, I_global)
        ])
        return np.array(D_out)
    
    def C(q, q_dot):
        D_num_diff = np.stack([partial_derivative(D, q, i) for i in range(len(q))], axis=0)
        C_matrix = 0.5 * (
            np.einsum('ikj,i->kj', D_num_diff, q_dot) +
            np.einsum('jki,i->kj', D_num_diff, q_dot) -
            np.einsum('kij,i->kj', D_num_diff, q_dot)
        )
        return C_matrix
    
    def P(q):
        rc_global = [(T @ np.array([*r_c, 1]))[:3] 
                     for T, r_c in zip(getDHTransformations(q, DHparameters)[1:], CenterOfMass[1:])]
        return sum([m_i * gravity.T @ r_c for m_i, r_c in zip(M, rc_global)])
    
    def g(q):
        return np.array([partial_derivative(P, q, i) for i in range(len(q))]).reshape(-1)

    g_vec = g(q)
    C_mat = C(q, q_dot)
    D_mat = D(q)
    tau = D_mat @ q_ddot + C_mat @ q_dot + g_vec
    return tau

def compute_torque_roboticstoolbox(q, q_dot, q_ddot, M, InertiaMatrices, CenterOfMass, DHparameters):
    """
    Compute joint torques using Peter Corke's Robotics Toolbox.
    """
    from roboticstoolbox import DHRobot, RevoluteDH
    n_dof = len(DHparameters)
    links = []
    for i in range(n_dof):
        # Create each link using DH parameters (theta offset is assumed zero here)
        link = RevoluteDH(d=DHparameters[i][1], a=DHparameters[i][2], alpha=DHparameters[i][3])
        # Set dynamic properties
        link.m = M[i]
        link.r = CenterOfMass[i]  # center of mass relative to the link frame
        link.I = InertiaMatrices[i]
        links.append(link)
    robot = DHRobot(links)
    gravity = np.array([0, 0, -9.81])
    tau = robot.rne(q, q_dot, q_ddot, gravity)
    return tau

def generate_random_dh(n_dof):
    return [
        [0.0,
         np.random.uniform(0.05, 0.5),    # d
         np.random.uniform(0.1, 1.0),     # a
         np.random.uniform(-np.pi, np.pi)]  # alpha
        for _ in range(n_dof)
    ]

def generate_random_dynamic_parameters(n_dof):
    # Masses for each of the n_dof links
    M = np.random.uniform(0.1, 5.0, n_dof).tolist()
    
    # InertiaMatrices for each link
    InertiaMatrices = []
    for _ in range(n_dof):
        I = np.diag(np.random.uniform(0.01, 1.0, 3))
        InertiaMatrices.append(I)
    
    # CenterOfMass for each link
    CenterOfMass = []
    for _ in range(n_dof):
        com = np.random.uniform(-0.1, 0.1, 3).tolist()
        CenterOfMass.append(com)
    
    DHparams = generate_random_dh(n_dof)
    
    # Random joint variables, velocities, accelerations
    q = np.random.uniform(-np.pi, np.pi, n_dof)
    q_dot = np.random.uniform(-1.0, 1.0, n_dof)
    q_ddot = np.random.uniform(-2.0, 2.0, n_dof)
    
    return M, InertiaMatrices, CenterOfMass, DHparams, q, q_dot, q_ddot

def run_single_iteration(dof):
    try:
        M, Inertia, CoM, DH, q, qd, qdd = generate_random_dynamic_parameters(dof)
        
        # Compute and time the C++ implementation
        start = time.perf_counter()
        tau_cpp = compute_torque.computeJointTorques(q, qd, qdd, M, Inertia, CoM, DH)
        time_cpp = time.perf_counter() - start
        
        # Compute and time the Python implementation 
        start = time.perf_counter()
        tau_py = computeJointTorques(q, qd, qdd, M, [np.zeros((3,3))] + Inertia, [np.zeros(3)] + CoM, DH)
        time_python = time.perf_counter() - start
        
        # Compute and time the Robotics Toolbox implementation
        start = time.perf_counter()
        tau_rtb = compute_torque_roboticstoolbox(q, qd, qdd, M, Inertia, CoM, DH)
        time_rtb = time.perf_counter() - start
        
        # Compute relative errors (using C++ output as reference)
        norm_cpp = np.linalg.norm(tau_cpp) + 1e-12
        error_cpp_py = np.linalg.norm(tau_cpp - tau_py) / norm_cpp
        error_cpp_rtb = np.linalg.norm(tau_cpp - tau_rtb) / norm_cpp
        
        return (time_cpp, time_python, time_rtb, error_cpp_py, error_cpp_rtb)
    except Exception as e:
        print(f"Error in DOF {dof}: {e}")
        return None

def benchmark_parallel(max_dof=32, runs_per_dof=100):
    # Test DOFs in powers of two up to max_dof
    dof_values = []
    power = 3
    while (2**power) <= max_dof:
        dof_values.append(2**power)
        power += 1
    
    results = {}
    with mp.Pool(processes=mp.cpu_count()) as pool:
        for dof in dof_values:
            print(f"Testing {dof} DOF...")
            cpp_times = []
            py_times = []
            rtb_times = []
            err_cpp_py = []
            err_cpp_rtb = []
            for outcome in pool.imap_unordered(
                partial(run_single_iteration),
                [dof] * runs_per_dof,
                chunksize=max(10, runs_per_dof // 10)
            ):
                if outcome is not None:
                    tc, tp, tr, ep, er = outcome
                    cpp_times.append(tc)
                    py_times.append(tp)
                    rtb_times.append(tr)
                    err_cpp_py.append(ep)
                    err_cpp_rtb.append(er)
            if cpp_times:
                results[dof] = (cpp_times, py_times, rtb_times, err_cpp_py, err_cpp_rtb)
            else:
                results[dof] = None
    return results

def plot_performance_line(results):
    plt.figure(figsize=(10, 6))
    sorted_dofs = sorted(d for d in results if results[d] is not None)
    
    dofs = []
    median_cpp, p5_cpp, p95_cpp = [], [], []
    median_py, p5_py, p95_py = [], [], []
    median_rtb, p5_rtb, p95_rtb = [], [], []
    
    for dof in sorted_dofs:
        cpp_times, py_times, rtb_times, _, _ = results[dof]
        cpp_ms = np.array(cpp_times) * 1000
        py_ms = np.array(py_times) * 1000
        rtb_ms = np.array(rtb_times) * 1000
        
        dofs.append(dof)
        median_cpp.append(np.median(cpp_ms))
        p5_cpp.append(np.percentile(cpp_ms, 5))
        p95_cpp.append(np.percentile(cpp_ms, 95))
        
        median_py.append(np.median(py_ms))
        p5_py.append(np.percentile(py_ms, 5))
        p95_py.append(np.percentile(py_ms, 95))
        
        median_rtb.append(np.median(rtb_ms))
        p5_rtb.append(np.percentile(rtb_ms, 5))
        p95_rtb.append(np.percentile(rtb_ms, 95))
    
    dofs = np.array(dofs)
    median_cpp = np.array(median_cpp)
    p5_cpp = np.array(p5_cpp)
    p95_cpp = np.array(p95_cpp)
    median_py = np.array(median_py)
    p5_py = np.array(p5_py)
    p95_py = np.array(p95_py)
    median_rtb = np.array(median_rtb)
    p5_rtb = np.array(p5_rtb)
    p95_rtb = np.array(p95_rtb)
    
    # Plot timing performance with error bars
    plt.errorbar(dofs, median_cpp, 
                 yerr=[median_cpp - p5_cpp, p95_cpp - median_cpp],
                 fmt='-o', capsize=4, label='C++')
    plt.errorbar(dofs, median_py, 
                 yerr=[median_py - p5_py, p95_py - median_py],
                 fmt='-s', capsize=4, label='Python')
    plt.errorbar(dofs, median_rtb, 
                 yerr=[median_rtb - p5_rtb, p95_rtb - median_rtb],
                 fmt='-^', capsize=4, label='Robotics Toolbox')
    
    plt.xscale('log', base=2)
    plt.yscale('log')
    plt.xticks(dofs, [str(d) for d in dofs])
    plt.xlim(dofs[0] / 1.5, dofs[-1] * 1.5)
    plt.xlabel('Degrees of Freedom [log2 Scale]')
    plt.ylabel('Computation Time (ms) [Median with 5th-95th Percentile]')
    plt.title('Torque Computation Performance: C++ vs. Python vs. Robotics Toolbox')
    plt.legend()
    plt.grid(True, which='both', linestyle=':')
    plt.tight_layout()
    plt.show()

def save_results_to_csv(results, filename="data/Benchmark_dynamics_result.csv"):
    """
    Saves the benchmark summary (timing and error statistics) to a CSV file.
    """
    sorted_dofs = sorted(d for d in results if results[d] is not None)
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["DOF", "Samples", 
                         "CPP_Median_ms", "CPP_5th_ms", "CPP_95th_ms",
                         "PY_Median_ms", "PY_5th_ms", "PY_95th_ms",
                         "RTB_Median_ms", "RTB_5th_ms", "RTB_95th_ms",
                         "CPP_PY_Error", "CPP_RTB_Error"])
        for dof in sorted_dofs:
            cpp_times, py_times, rtb_times, err_cpp_py, err_cpp_rtb = results[dof]
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
                np.percentile(rtb_ms, 95),
                np.median(err_cpp_py),
                np.median(err_cpp_rtb)
            ])

if __name__ == "__main__":
    max_dof = 2**5  # e.g., up to 128 DOF
    runs_per_dof = 100
    
    results = benchmark_parallel(max_dof, runs_per_dof)
    
    # Print summary to screen (including error statistics)
    print("\nBenchmark Summary:")
    header = ("DOF", "Samples", 
              "C++ Median (ms)", "C++ p95 (ms)", 
              "Python Median (ms)", "Python p95 (ms)",
              "RTB Median (ms)", "RTB p95 (ms)",
              "C++ vs Python Error", "C++ vs RTB Error")
    print(" | ".join(f"{h:<20}" for h in header))
    for dof in sorted(results.keys()):
        if results[dof] is not None:
            cpp, py, rtb, err_cpp_py, err_cpp_rtb = results[dof]
            cpp_ms = np.array(cpp) * 1000
            py_ms = np.array(py) * 1000
            rtb_ms = np.array(rtb) * 1000
            print(" | ".join([
                f"{dof:<20}",
                f"{len(cpp):<20}",
                f"{np.median(cpp_ms):<20.3f}",
                f"{np.percentile(cpp_ms, 95):<20.3f}",
                f"{np.median(py_ms):<20.3f}",
                f"{np.percentile(py_ms, 95):<20.3f}",
                f"{np.median(rtb_ms):<20.3f}",
                f"{np.percentile(rtb_ms, 95):<20.3f}",
                f"{np.median(err_cpp_py):<20.3e}",
                f"{np.median(err_cpp_rtb):<20.3e}"
            ]))
    
    plot_performance_line(results)
    save_results_to_csv(results, filename="Benchmark/Benchmark_dynamics_result.csv")
