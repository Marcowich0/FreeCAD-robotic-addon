import FreeCAD
import FreeCADGui
import os
import numpy as np
from PySide import QtCore

import math


# For plotting:
from scipy.interpolate import splprep, splev  # example import if needed
from Utils.main_utils import *
from inverse_kinematics import solve_ik
import inverse_kinematics_cpp

# Import parent class
from Trajectory_parent import Trajectory, ViewProviderTrajectory, selectEdgesForTrajectory, computeTrajectoryPoints


class ConstantVelocityTrajectory(Trajectory):
    """
    Child class of Trajectory that contains all the existing logic
    for solving the trajectory at (nominally) constant velocity and saving data.
    """
    def __init__(self, obj):
        super().__init__(obj)
        # Add properties specific to this trajectory type

        obj.addProperty("App::PropertyString", "SubType", "Trajectory", "Type of the object").SubType = "ConstantVelocity"

        obj.addProperty("App::PropertyLink", "Body", "Trajectory", "Link to a Body")
        obj.addProperty("App::PropertyLinkSubList", "Edges", "Trajectory", "Link to Edges").Edges = []

        obj.addProperty("App::PropertyFloat", "Velocity", "Trajectory", "Velocity").Velocity = 0.1

        # Spline/tuning parameters
        obj.addProperty("App::PropertyFloat", "smoothing", "Trajectory", "Smoothing factor").smoothing = 100
        obj.addProperty("App::PropertyFloat", "alpha", "Trajectory", "Pre-path extension").alpha = 0.1

        obj.addProperty("App::PropertyFloat", "tangential", "EndEffectorLoad", "Tangential force").tangential = 0.0
        obj.addProperty("App::PropertyFloat", "normal", "EndEffectorLoad", "Normal force").normal = 0.0
        obj.addProperty("App::PropertyFloat", "torque", "EndEffectorLoad", "Torque").torque = 0.0

        obj.addProperty("App::PropertyBool", "externalModel", "Trajectory", "External Model").externalModel = False
        obj.addProperty("App::PropertyFloat", "DistanceBetweenPoints", "Trajectory", "Distance between points").DistanceBetweenPoints = 5e-3

    
    from freecad_robotics.compute_torque import computeJointTorques

    def solve(self):
        """
        Solve trajectory at constant velocity, compute tangents & normals
        via finite differences (no divide-by-zero), apply loads via Jacobian,
        add to dynamics torques, and store everything.
        """
        obj   = self.Object
        robot = get_robot()

        # -- Setup ------------------------------------------------------------
        DH         = getNumericalDH()
        target_dir = vec_to_numpy(robot.EndEffectorOrientation)
        target_dir /= np.linalg.norm(target_dir)  # unit

        # -- 1) Spline through raw points ------------------------------------
        raw_pts = [vec_to_numpy(p) / 1000.0 for p in computeTrajectoryPoints(obj)]
        if len(raw_pts) < 2:
            print("Not enough points!")
            return
        pts_np = np.array(raw_pts).T  # (3, N)

        tck, _     = splprep(pts_np, s=(obj.smoothing * 1e-6))
        num_sample = len(raw_pts) * 4
        u_new      = np.linspace(-obj.alpha, 1.0, num_sample)
        smooth_pts = np.array(splev(u_new, tck, ext=3)).T  # (M, 3)

        from FreeCAD import Vector
        obj.SplinePoints = [Vector(*pt) for pt in smooth_pts]

        # -- 2) Arc-length → time --------------------------------------------
        M = len(smooth_pts)
        arc = np.zeros(M)
        for i in range(1, M):
            arc[i] = arc[i-1] + np.linalg.norm(smooth_pts[i] - smooth_pts[i-1])
        t_array = arc / obj.Velocity  # (M,)

        # -- 3) Compute unit-tangent via finite differences -----------------
        # differences between adjacent points
        delta_pts = smooth_pts[1:] - smooth_pts[:-1]          # (M-1,3)
        dists     = np.linalg.norm(delta_pts, axis=1, keepdims=True)
        dists     = np.where(dists > 1e-8, dists, 1.0)
        unit_segs = delta_pts / dists                         # (M-1,3)

        tangent_arr = np.zeros_like(smooth_pts)               # (M,3)
        tangent_arr[0]    = unit_segs[0]
        tangent_arr[-1]   = unit_segs[-1]
        tangent_arr[1:-1] = 0.5 * (unit_segs[:-1] + unit_segs[1:])

        # -- 4) Compute normals via cross(tangent, target_dir) ------------
        normals_unnorm = np.cross(tangent_arr, target_dir[np.newaxis, :])  # (M,3)
        norms          = np.linalg.norm(normals_unnorm, axis=1, keepdims=True)
        norms          = np.where(norms > 1e-8, norms, 1.0)
        normal_arr     = normals_unnorm / norms                           # (M,3)

        # -- 5) Read load magnitudes -----------------------------------------
        F_t_mag = obj.tangential
        F_n_mag = obj.normal
        M_m_mag = obj.torque

        # -- 6) Build wrenches [F; M] at each sample -------------------------
        wrenches = np.zeros((M, 6))
        for i in range(M):
            f_vec = F_t_mag * tangent_arr[i] + F_n_mag * normal_arr[i]
            m_vec = M_m_mag * target_dir
            wrenches[i, :3] = f_vec
            wrenches[i, 3:] = m_vec

        # -- 7) Inverse Kinematics --------------------------------------------
        q_list = []
        q0     = np.deg2rad(robot.Angles)
        for i, P in enumerate(smooth_pts):
            if i == 0:
                q = solve_ik(q0, P, target_dir, DH)
            else:
                _, q = inverse_kinematics_cpp.solveIK(q, P, target_dir, DH)
            q_list.append(q)
        q_arr = np.array(q_list)  # (M, n_joints)

        # -- 8) Kinematic derivatives ----------------------------------------
        # note: these still use gradient w.r.t t_array but rarely trigger zero Δt now
        q_dot_arr  = np.gradient(q_arr, t_array, axis=0)
        q_ddot_arr = np.gradient(q_dot_arr, t_array, axis=0)

        # -- 9) Store kinematics on obj --------------------------------------
        obj.t      = t_array.tolist()
        obj.q      = q_arr.tolist()
        obj.q_dot  = q_dot_arr.tolist()
        obj.q_ddot = q_ddot_arr.tolist()

        # -- 10) Dynamics torques --------------------------------------------
        super().updateTorques()
        dyn_tau = np.array(obj.Torques)  # (M, n_joints)

        # -- 11) Map wrenches → joint torques via Jacobian -------------------
        tau_force = np.zeros_like(dyn_tau)
        for i, qi in enumerate(q_list):
            J       = inverse_kinematics_cpp.getJacobian(qi, DH)  # (6×n)
            tau_i   = J.T.dot(wrenches[i])                        # (n,)
            tau_force[i] = tau_i

        # -- 12) Sum and store total torques ---------------------------------
        total_tau   = dyn_tau + tau_force
        obj.Torques = total_tau.tolist()

        # -- 13) Update robot to final pose ---------------------------------
        robot.Angles = np.rad2deg(q_list[-1]).tolist()

        print("Trajectory solved with corrected tangent/normal computation and applied loads.")


#
# --------------------- COMMANDS ---------------------
#
class AddConstantVelTrajectoryCommand:
    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(os.path.dirname(__file__)), 'Resources', 'icons', 'trajectory_constant_velocity.svg'),
            'MenuText': 'Add Constant Velocity Trajectory',
            'ToolTip': 'Add a trajectory object for a path with constant velocity'
        }

    def Activated(self):
        doc = FreeCAD.activeDocument()
        trajectory_obj = doc.addObject("App::FeaturePython", "Trajectory")
        doc.Assembly.addObject(trajectory_obj)

        # Attach the parent or child class proxy - your choice:
        # If you want the 'ConstantVelocityTrajectory' object, do:
        ConstantVelocityTrajectory(trajectory_obj)

        trajectory_obj.ViewObject.Proxy = ViewProviderTrajectory(trajectory_obj.ViewObject)

        # Instead of calculating points now, only store the edge (and parent body)
        selectEdgesForTrajectory(trajectory_obj)

        doc.recompute()

    def IsActive(self):
        return bool(currentSelectionType() == 'Edge' and get_robot())


FreeCADGui.addCommand('AddConstantVelTrajectoryCommand', AddConstantVelTrajectoryCommand())




