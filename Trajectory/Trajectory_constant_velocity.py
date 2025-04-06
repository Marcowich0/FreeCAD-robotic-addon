import FreeCAD
import FreeCADGui
import os
import numpy as np
from PySide import QtCore
import csv
import math

# For file dialogs:
from PySide import QtGui

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
        obj.addProperty("App::PropertyVectorList", "SplinePoints", "Trajectory", "List of spline points").SplinePoints = []

        obj.addProperty("App::PropertyFloat", "Velocity", "Trajectory", "Velocity").Velocity = 0.1

        # Spline/tuning parameters
        obj.addProperty("App::PropertyFloat", "smoothing", "Trajectory", "Smoothing factor").smoothing = 100
        obj.addProperty("App::PropertyFloat", "alpha", "Trajectory", "Pre-path extension").alpha = 0.1

        obj.addProperty("App::PropertyBool", "externalModel", "Trajectory", "External Model").externalModel = False
        obj.addProperty("App::PropertyFloat", "DistanceBetweenPoints", "Trajectory", "Distance between points").DistanceBetweenPoints = 5e-3

    
    def solve(self):
        """
        Solve the trajectory at (nominally) constant velocity using the same
        logic originally in solvePath(), but referencing self.Object 
        instead of the local variable `sel`.
        """

        obj = self.Object
        robot = get_robot()

        DH = getNumericalDH()
        target_dir = vec_to_numpy(robot.EndEffectorOrientation)

        # 1) Get raw 3D points (in meters) from object
        raw_pts = [vec_to_numpy(p)/1000.0 for p in computeTrajectoryPoints(obj)]
        if len(raw_pts) < 2:
            print("Not enough points!")
            return
        pts_np = np.array(raw_pts).T  # shape (3, N)

        # 2) Fit a parametric spline
        tck, u = splprep(pts_np, s=(obj.smoothing * 1e-6))
        num_sample = len(raw_pts) * 4
        u_new = np.linspace(-obj.alpha, 1, num_sample)
        smooth_pts = np.array(splev(u_new, tck, ext=3)).T  # shape (M, 3)

        # Store spline points on the FreeCAD object
        from FreeCAD import Vector
        obj.SplinePoints = [Vector(*pt) for pt in smooth_pts]

        # 3) Arc-length to time
        arc = np.zeros(len(smooth_pts))
        for i in range(1, len(smooth_pts)):
            arc[i] = arc[i-1] + np.linalg.norm(smooth_pts[i] - smooth_pts[i-1])
        t_array = arc / obj.Velocity

        # 4) Derivatives (Cartesian space)
        x_dot = np.gradient(smooth_pts, t_array, axis=0)

        # 5) Inverse Kinematics
        q_list = []
        q = np.deg2rad(robot.Angles)  # initial guess
        for i, p in enumerate(smooth_pts):
            if i == 0:
                # Possibly your custom solve_ik or initial solve
                q = solve_ik(q, p, target_dir, DH)
            else:
                _, q = inverse_kinematics_cpp.solveIK(q, p, target_dir, DH)
            q_list.append(q)


        q_arr = np.array(q_list)
        q_dot_arr = np.gradient(q_arr, t_array, axis=0)
        q_ddot_arr = np.gradient(q_dot_arr, t_array, axis=0)

        # 6) Store results in the object
        obj.t      = np.array(t_array).tolist()
        obj.q      = np.array(q_list).tolist()
        obj.q_dot  = np.array(q_dot_arr).tolist()
        obj.q_ddot = np.array(q_ddot_arr).tolist()

        # Robot's final angles (degrees)
        robot.Angles = np.rad2deg(q_list[-1]).tolist()

        print("Trajectory solved with parametric smoothing and pre-path extension.")
        displayMatrix(obj.t)  # if you still want this for debugging/logging

        self.updateTorques()


#
# --------------------- COMMANDS ---------------------
#
class AddConstantVelTrajectoryCommand:
    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(os.path.dirname(__file__)), 'Resources', 'icons', 'trajectory.svg'),
            'MenuText': 'Add Trajectory',
            'ToolTip': 'Add a trajectory object'
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





class SaveTrajectoryDataCommand:
    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(os.path.dirname(__file__)), 'Resources', 'icons', 'save.svg'),
            'MenuText': 'Save Trajectory Data',
            'ToolTip': (
                'Save trajectory time, angles (converted to radians in [-π, π] with jump filtering), '
                'velocities, accelerations, torques, along with both the spline path and the original points. '
                'Data is downsampled to reduce file size.'
            )
        }

    def Activated(self):
        sel_list = FreeCADGui.Selection.getSelection()
        if not sel_list:
            print("No object selected!")
            return

        traj_obj = sel_list[0]
        if not hasattr(traj_obj, 'Type') or traj_obj.Type != 'Trajectory':
            print("Selected object is not a trajectory!")
            return

        try:
            t = list(traj_obj.t)
            angles = list(traj_obj.q)
            q_dot = list(traj_obj.q_dot)
            q_ddot = list(traj_obj.q_ddot)
            torques = list(traj_obj.Torques)
            orig_points = traj_obj.Points
            spline_points = traj_obj.SplinePoints
        except Exception as e:
            print("Error accessing trajectory data:", e)
            return

        # Convert FreeCAD.Vectors to lists (x, y, z)
        orig_points_list = []
        for pt in orig_points:
            if hasattr(pt, 'x'):
                orig_points_list.append([pt.x/1000, pt.y/1000, pt.z/1000])
            else:
                orig_points_list.append(pt)

        spline_points_list = []
        for pt in spline_points:
            if hasattr(pt, 'x'):
                spline_points_list.append([pt.x, pt.y, pt.z])
            else:
                spline_points_list.append(pt)

        n_rows = len(t)
        if not (len(angles) == len(q_dot) == len(q_ddot) == len(torques) == n_rows):
            print("Mismatch in main trajectory data lengths!")
            return

        max_t = max(t) if max(t) != 0 else 1

        def get_num_joints(data):
            if isinstance(data[0], (list, tuple, np.ndarray)):
                return len(data[0])
            else:
                return 1

        num_joints_angles = get_num_joints(angles)
        num_joints_q_dot = get_num_joints(q_dot)
        num_joints_q_ddot = get_num_joints(q_ddot)
        num_joints_torques = get_num_joints(torques)

        def convert_angle(angle_deg):
            rad = math.radians(angle_deg)
            return ((rad + math.pi) % (2*math.pi)) - math.pi

        def angle_diff(a, b):
            return abs(a - b)

        threshold = 1.0
        downsample_factor = 5
        prev_angles = [None]*num_joints_angles

        header = ["time", "time_norm"]
        header += [f"angle_{i+1}" for i in range(num_joints_angles)]
        header += [f"q_dot_{i+1}" for i in range(num_joints_q_dot)]
        header += [f"q_ddot_{i+1}" for i in range(num_joints_q_ddot)]
        header += [f"torque_{i+1}" for i in range(num_joints_torques)]

        main_fileName, _ = QtGui.QFileDialog.getSaveFileName(None, "Save Trajectory Data", "", "CSV Files (*.csv)")
        if not main_fileName:
            print("No file selected; data not saved.")
            return

        try:
            with open(main_fileName, "w", newline="") as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(header)
                for i in range(0, n_rows, downsample_factor):
                    if float(t[i]) == 0.0:
                        continue
                    row = [t[i], t[i]/max_t]
                    # angles
                    if num_joints_angles > 1:
                        converted_angles = []
                        for j, a in enumerate(angles[i]):
                            new_angle = convert_angle(a)
                            if prev_angles[j] is not None:
                                if angle_diff(new_angle, prev_angles[j]) > threshold:
                                    converted_angles.append(float('nan'))
                                else:
                                    converted_angles.append(new_angle)
                                prev_angles[j] = new_angle
                            else:
                                prev_angles[j] = new_angle
                                converted_angles.append(new_angle)
                        row.extend(converted_angles)
                    else:
                        new_angle = convert_angle(angles[i])
                        if prev_angles[0] is not None:
                            if angle_diff(new_angle, prev_angles[0]) > threshold:
                                row.append(float('nan'))
                            else:
                                row.append(new_angle)
                            prev_angles[0] = new_angle
                        else:
                            prev_angles[0] = new_angle
                            row.append(new_angle)
                    # velocities
                    if num_joints_q_dot > 1:
                        row.extend(q_dot[i])
                    else:
                        row.append(q_dot[i])
                    # accelerations
                    if num_joints_q_ddot > 1:
                        row.extend(q_ddot[i])
                    else:
                        row.append(q_ddot[i])
                    # torques
                    if num_joints_torques > 1:
                        row.extend(torques[i])
                    else:
                        row.append(torques[i])
                    writer.writerow(row)
            print("Main trajectory data successfully saved as CSV to:", main_fileName)
        except Exception as e:
            print("Failed to save main trajectory data as CSV:", e)
            return

        # Save original points
        base, ext = os.path.splitext(main_fileName)
        orig_fileName = base + "_orig_points" + ext
        try:
            with open(orig_fileName, "w", newline="") as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(["point_x", "point_y", "point_z"])
                for pt in orig_points_list:
                    if len(pt) == 3:
                        writer.writerow(pt)
                    else:
                        writer.writerow([None, None, None])
            print("Original trajectory points successfully saved as CSV to:", orig_fileName)
        except Exception as e:
            print("Failed to save original trajectory points as CSV:", e)

        # Save spline points
        spline_fileName = base + "_spline_points" + ext
        try:
            with open(spline_fileName, "w", newline="") as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(["point_x", "point_y", "point_z"])
                for pt in spline_points_list:
                    if len(pt) == 3:
                        writer.writerow(pt)
                    else:
                        writer.writerow([None, None, None])
            print("Spline trajectory points successfully saved as CSV to:", spline_fileName)
        except Exception as e:
            print("Failed to save spline trajectory points as CSV:", e)

    def IsActive(self):
        sel_list = FreeCADGui.Selection.getSelection()
        if not sel_list:
            return False
        sel = sel_list[0]
        return hasattr(sel, 'Type') and sel.Type == 'Trajectory'


FreeCADGui.addCommand('SaveTrajectoryDataCommand', SaveTrajectoryDataCommand())
