import FreeCAD
import FreeCADGui
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import numpy as np
import matplotlib.pyplot as plt
import os
import argparse

from Utils.main_utils import *

import inverse_kinematics_cpp
import compute_torque

from Trajectory_parent import Trajectory, ViewProviderTrajectory, selectEdgesForTrajectory, computeTrajectoryPoints


class TimeOptimizedTrajectory(Trajectory):
    """
    Child class that contains the logic for solving a time-optimized trajectory.
    """
    def __init__(self, obj):
        super().__init__(obj)
        obj.addProperty("App::PropertyString", "SubType", "Trajectory", "Type of the object").SubType = "TimeOptimized"
        obj.addProperty("App::PropertyFloatList", "VelocityLimits", "Constrains", "Velocity limits").VelocityLimits = [10 for _ in get_robot().Angles]
        obj.addProperty("App::PropertyFloatList", "AccelerationLimits", "Constrains", "Acceleration limits").AccelerationLimits = [25 for _ in get_robot().Angles]
        obj.addProperty("App::PropertyFloatList", "TorqueLimits", "Constrains", "Torque limits").TorqueLimits = [10 for _ in get_robot().Angles]

    def solve(self):
        """
        Solve the trajectory using TOPPRA, enforcing joint-angle, velocity, and acceleration limits.
        """

        obj = self.Object
        robot = get_robot()

        if obj.SubsubType == "TimeOptimizedLine":
            computeTrajectoryPoints(obj)

        # Build joint waypoints via inverse kinematics
        q0 = np.deg2rad(robot.Angles)
        global_points = [vec_to_numpy(p) / 1000 for p in obj.Points]
        orientation = vec_to_numpy(robot.EndEffectorOrientation)
        DH = getNumericalDH()

        q_sol = []
        for i, point in enumerate(global_points):
            if i == 0:
                q_sol.append(
                    inverse_kinematics_cpp.solveIK(q0, point, orientation, DH)[1]
                )
            else:
                q_sol.append(
                    inverse_kinematics_cpp.solveIK(q_sol[i - 1],
                                                   point,
                                                   orientation,
                                                   DH)[1]
                )
        q_waypoints = np.array(q_sol)

        # Retrieve limits from the FreeCAD properties
        vel_limits   = np.array(obj.VelocityLimits)      # deg/s or rad/s as you prefer
        accel_limits = np.array(obj.AccelerationLimits)  # deg/s² or rad/s²
        torque_limits = np.array(obj.TorqueLimits)     # Nm

        # Compute time-optimal trajectory with TOPP-RA
        t_path, q_path, qd_path, qdd_path = Toppra(
            q_waypoints,
            vel_limits,
            accel_limits,
            torque_limits,
            N=1000
        )


        # Store results back into the object
        obj.t       = t_path.tolist()
        obj.q       = q_path.tolist()
        obj.q_dot   = qd_path.tolist()
        obj.q_ddot  = qdd_path.tolist()

        # Recompute torque if needed
        self.updateTorques()

        return



class TimeOptimizedTrajectoryLine(TimeOptimizedTrajectory):
    """
    Trajectory based on a line (edge).
    """
    def __init__(self, obj):
        super().__init__(obj)
        obj.addProperty("App::PropertyString", "SubsubType", "Trajectory", "Type of the object").SubsubType = "TimeOptimizedLine"
        obj.addProperty("App::PropertyLink", "Body", "Trajectory", "Link to a Body").Body = None
        obj.addProperty("App::PropertyLinkSubList", "Edges", "Trajectory", "Link to Edges").Edges = []
        obj.addProperty("App::PropertyFloat", "DistanceBetweenPoints", "Trajectory", "Distance between points").DistanceBetweenPoints = 5e-3
        obj.addProperty("App::PropertyBool", "externalModel", "Trajectory", "External Model").externalModel = False


class TimeOptimizedTrajectoryPoints(TimeOptimizedTrajectory):
    """
    Trajectory based on selected points.
    """
    def __init__(self, obj):
        super().__init__(obj)
        obj.addProperty("App::PropertyString", "SubsubType", "Trajectory", "Type of the object").SubsubType = "TimeOptimizedPoints"
        obj.addProperty("App::PropertyLink", "Body", "Trajectory", "Link to a Body")
        obj.addProperty("App::PropertyLinkSubList", "ScourcePoints", "Trajectory", "Link to Edges").ScourcePoints = []


#
# --------------------- COMMANDS ---------------------
#
class AddTimeOptimizedTrajectoryCommand:
    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(os.path.dirname(__file__)), 'Resources', 'icons', 'trajectory_time_optimized.svg'),
            'MenuText': 'Add Time Optimized Trajectory',
            'ToolTip': 'Add a trajectory object for time optimized path planning'
        }

    def Activated(self):
        doc = FreeCAD.activeDocument()
        trajectory_obj = doc.addObject("App::FeaturePython", "Trajectory")
        doc.Assembly.addObject(trajectory_obj)

        sel_type = currentSelectionType()

        if sel_type == 'Edge':
            TimeOptimizedTrajectoryLine(trajectory_obj)
            trajectory_obj.ViewObject.Proxy = ViewProviderTrajectory(trajectory_obj.ViewObject)
            selectEdgesForTrajectory(trajectory_obj)
        elif sel_type == 'Vertex':
            TimeOptimizedTrajectoryPoints(trajectory_obj)
            trajectory_obj.ViewObject.Proxy = ViewProviderTrajectory(trajectory_obj.ViewObject)
            source_points = get_source_points()
            trajectory_obj.ScourcePoints = source_points
            trajectory_obj.Points = get_global_points()
        else:
            FreeCAD.Console.PrintError("Selected object is not a valid trajectory type.\n")
            return

        doc.recompute()

    def IsActive(self):
        return bool((currentSelectionType() == 'Edge' or currentSelectionType() == 'Vertex') and get_robot())


FreeCADGui.addCommand('AddTimeOptimizedTrajectoryCommand', AddTimeOptimizedTrajectoryCommand())


def get_global_points():
    sel = FreeCADGui.Selection.getSelectionEx()
    points = [vec_to_numpy(p) for p in sel[0].PickedPoints]
    ordered_points = []
    current_point = points.pop(0)
    ordered_points.append(current_point)

    while points:
        distances = [np.linalg.norm(p - current_point) for p in points]
        next_idx = np.argmin(distances)
        next_point = points.pop(next_idx)
        ordered_points.append(next_point)
        current_point = next_point

    return [numpy_to_vec(p) for p in ordered_points]


def get_source_points():
    sel = FreeCADGui.Selection.getSelectionEx()
    points = []
    for sel_obj in sel:
        obj = sel_obj.Object
        for sub_name in sel_obj.SubElementNames:
            if "Vertex" in sub_name:
                points.append((obj, sub_name))
    return points


def Toppra(way_pts, vel_limits, accel_limits, torque_limits, N=100):
    import numpy as np
    import toppra as ta
    import toppra.constraint as constraint
    import toppra.algorithm as algo

    # 1) Build the path interpolator
    path_scalars = np.linspace(0, 1, len(way_pts))
    path = ta.SplineInterpolator(path_scalars, way_pts)

    # 2) Velocity & acceleration constraints
    vlim = np.vstack((-vel_limits,    vel_limits)).T
    alim = np.vstack((-accel_limits,  accel_limits)).T
    pc_vel = constraint.JointVelocityConstraint(vlim)
    pc_acc = constraint.JointAccelerationConstraint(
        alim,
        discretization_scheme=constraint.DiscretizationType.Interpolation
    )

    # 3) torque_limits is accepted but ignored

    # 4) Create and solve the TOPP-RA instance (positional args!)
    #     First arg: list of constraints
    #     Second arg: the path object
    instance = algo.TOPPRA([pc_vel, pc_acc], path, solver_wrapper='seidel')
    jnt_traj = instance.compute_trajectory(0, 0)

    # 5) Sample the resulting trajectory
    ts_sample   = np.linspace(0, jnt_traj.get_duration(), N)
    qs_sample   = jnt_traj.eval(ts_sample)
    qds_sample  = jnt_traj.evald(ts_sample)
    qdds_sample = jnt_traj.evaldd(ts_sample)

    return ts_sample, qs_sample, qds_sample, qdds_sample
