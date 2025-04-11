import sys
import os
import inspect
import numpy as np
import matplotlib.pyplot as plt
from Utils.main_utils import *

# Path configuration: adjust this so that the C++ module is accessible.
current_dir = os.path.dirname(os.path.dirname(os.path.abspath(inspect.getsourcefile(lambda:0))))
module_path = os.path.join(current_dir, "robot_dynamics_module", "build", "Release")
sys.path.append(module_path)

import robot_dynamics_module.build.Release.compute_torque as compute_torque  # C++ module for torque computation


# ------------------------------
# Compute joint torques using Peter Corke's Robotics Toolbox
# ------------------------------
def compute_torque_roboticstoolbox(q, q_dot, q_ddot, M, InertiaMatrices, CenterOfMass, DH_params):
    from roboticstoolbox import DHRobot, RevoluteDH
    n_dof = len(DH_params)
    links = []
    for i in range(n_dof):
        # Create each link using the DH parameters
        link = RevoluteDH(d=DH_params[i][1], a=DH_params[i][2], alpha=DH_params[i][3])
        # Set dynamic properties
        link.m = M[i]
        link.r = CenterOfMass[i]
        link.I = InertiaMatrices[i]
        links.append(link)
    robot = DHRobot(links)
    gravity = np.array([0, 0, -9.81])
    tau = robot.rne(q, q_dot, q_ddot, gravity)
    return tau









# ------------------------------
# Define the robot parameters (2-link robot)
# ------------------------------

# Each row of DH_params: [theta_offset, d, a, alpha]
DH_params = [
    [0.0, 0.0, 0.0, np.pi/2], 
    [0.0, 0.0, 0.0, 0]
]

# Dynamic parameters:
# Masses: assume m1 and m2 (for a 2-link robot)
M = np.array([2 for _ in DH_params])  # Masses of each link
m = 2
l = 3

# Inertia matrices: for simplicity use diagonal matrices.
I = 1/12 * m * l**2
InertiaMatrices = [
    np.array([[I, 0, 0], [0, I, 0], [0, 0, 0]]),
    np.array([[I, 0, 0], [0, 0, 0], [0, 0, I]])
]

# Center of mass:
CenterOfMass = np.array([
    [0, 0, l/2],    # For Joint 1
    [0, l/2, l]  # Link 2
])


# ------------------------------
# Define joint states (2 DOF)
# ------------------------------
# For plotting, we set a configuration. Here we choose the zero configuration.
q = np.array([np.pi/4,np.pi/4])
q_dot = np.array([10, 10])
q_ddot = np.array([2,5])


tau_cpp = compute_torque.computeJointTorques(
    q, q_dot, q_ddot, 
    M, InertiaMatrices, CenterOfMass, 
    np.array(DH_params)
)


tau_rtb = compute_torque_roboticstoolbox(q, q_dot, q_ddot, M, InertiaMatrices, CenterOfMass, DH_params)

# ------------------------------
# Nicely print the torque results
# ------------------------------
def print_torques(label, tau):
    print(f"{label} Torque Output:")
    for i, t in enumerate(tau):
        print(f"  Joint {i+1}: {t: .6f}")
    print()

print("Planar Robot Experiment (Static Configuration)")
print("================================================")
print("Joint configuration (q) in radians:", q)
print("Joint velocities (q_dot):", q_dot)
print("Joint accelerations (q_ddot):", q_ddot)
print("Matrices of Inertia")
for I in InertiaMatrices:
    displayMatrix(I) 

print_torques("C++", tau_cpp)
print_torques("Robotics Toolbox", tau_rtb)

norm_cpp = np.linalg.norm(tau_cpp) + 1e-12
rel_error_rtb = np.linalg.norm(tau_cpp - tau_rtb) / norm_cpp
print("Relative error (C++ vs Robotics Toolbox): {:.6e}".format(rel_error_rtb))

