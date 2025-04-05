import FreeCAD
import FreeCADGui
import Part
import os
import numpy as np
from PySide import QtCore
import threading
import cProfile
import pstats
import csv
import math

# For file dialogs:
from PySide import QtGui

# For plotting:
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline, splprep, splev
# If you have your own custom modules:
from dynamics import computeJointTorques
from main_utils import currentSelectionType, get_robot, displayMatrix, vec_to_numpy, mat_to_numpy
from inverse_kinematics import solve_ik
import inverse_kinematics_cpp
from forward_kinematics import getJacobian
import compute_torque

# Import parent class
from Trajectory_parent import Trajectory, ViewProviderTrajectory


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

        obj.addProperty("App::PropertyFloat", "Velocity", "Trajectory", "Velocity").Velocity = 1

        # Spline/tuning parameters
        obj.addProperty("App::PropertyFloat", "smoothing", "Trajectory", "Smoothing factor").smoothing = 1
        obj.addProperty("App::PropertyFloat", "alpha", "Trajectory", "Pre-path extension").alpha = 0.1

        obj.addProperty("App::PropertyBool", "externalModel", "Trajectory", "External Model").externalModel = False
        obj.addProperty("App::PropertyFloat", "DistanceBetweenPoints", "Trajectory", "Distance between points").DistanceBetweenPoints = 5e-3

    
    def solve(self):
        """
        Solve the trajectory at (nominally) constant velocity using the same
        logic originally in solvePath(), but referencing self.Object 
        instead of the local variable `sel`.
        """
        from FreeCADGui import Selection
        from scipy.interpolate import splprep, splev  # example import if needed

        obj = self.Object
        robot = get_robot()

        # Prepare DH parameters
        DHraw = np.array(robot.DHPerameters, dtype=object)
        DHraw[:, 0] = 0.0
        DH = DHraw.astype(float)
        DH[:, 1:3] /= 1000  # convert mm to m
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
                converged, q = inverse_kinematics_cpp.solveIK(q, p, target_dir, DH)
            q_list.append(q)

        # 6) Joint velocities with Jacobian
        q_dot_list = []
        for i, q_now in enumerate(q_list):
            J = getJacobian(q_now)
            if J.ndim > 2:
                J = J[0]
            J_pos = J[0:3, :]
            q_dot_list.append(np.linalg.pinv(J_pos).dot(x_dot[i]))
        q_dot_arr = np.array(q_dot_list)
        q_ddot_arr = np.gradient(q_dot_arr, t_array, axis=0)

        # 7) Store results in the object
        obj.q      = q_list
        obj.t      = t_array
        obj.q_dot  = q_dot_arr
        obj.q_ddot = q_ddot_arr

        # Robot's final angles (degrees)
        robot.Angles = np.rad2deg(q_list[-1]).tolist()

        print("Trajectory solved with parametric smoothing and pre-path extension.")
        displayMatrix(obj.t)  # if you still want this for debugging/logging

        self.updateTorques()

#
# --------------------- HELPER FUNCTIONS ---------------------
#
def selectEdgesForTrajectory(trajectory_obj):
    collected_edges = []
    for sel in FreeCADGui.Selection.getSelectionEx():
        for subel_name in sel.SubElementNames:
            collected_edges.append((sel.Object, subel_name))

    trajectory_obj.Edges = collected_edges

    # Attempt to set 'Body' by searching parents
    if collected_edges:
        obj = collected_edges[0][0]
        for parent in obj.Parents:
            if parent[0].Name == 'Assembly':
                body_name = parent[1].split('.')[0]
                print(f"Found parent body: {body_name}")
                trajectory_obj.Body = FreeCAD.ActiveDocument.getObject(body_name)
                break

    print(f"Collected {len(collected_edges)} sub-elements in total.")


def chain_lists(list_of_lists):
    arrs = list_of_lists
    chain = arrs.pop(0)  # take the first
    while arrs:
        best_dist = float('inf')
        best_idx = -1
        best_flip = False
        best_prepend = False
        for i, seg in enumerate(arrs):
            cs, ce = chain[0], chain[-1]  # current chain start/end
            ss, se = seg[0], seg[-1]      # segment start/end
            # check 4 combos
            d1 = np.linalg.norm(ce - ss)  # append normal
            if d1 < best_dist:
                best_dist, best_idx, best_flip, best_prepend = d1, i, False, False
            d2 = np.linalg.norm(ce - se)  # append reversed
            if d2 < best_dist:
                best_dist, best_idx, best_flip, best_prepend = d2, i, True, False
            d3 = np.linalg.norm(cs - ss)  # prepend reversed
            if d3 < best_dist:
                best_dist, best_idx, best_flip, best_prepend = d3, i, True, True
            d4 = np.linalg.norm(cs - se)  # prepend normal
            if d4 < best_dist:
                best_dist, best_idx, best_flip, best_prepend = d4, i, False, True
        chosen = arrs.pop(best_idx)
        if best_flip:
            chosen.reverse()
        if best_prepend:
            chain = chosen[:-1] + chain
        else:
            chain = chain + chosen[1:]
    return [FreeCAD.Vector(*p) for p in chain]


def computeTrajectoryPoints(trajectory_obj):
    # We assume there's at least one selected Trajectory in the GUI
    sel = FreeCADGui.Selection.getSelection()[0]
    if not trajectory_obj.Edges:
        print("Error: No edge stored in the trajectory object!")
        return []

    point_list = []
    for (obj, sub_elems) in trajectory_obj.Edges:
        # sub_elems might be a single name or a list
        names = sub_elems if isinstance(sub_elems, (list, tuple)) else [sub_elems]
        for sub in names:
            print("edge array", obj, sub)
            edge_candidate = obj.getSubObject(sub)

            if isinstance(edge_candidate, tuple):
                edge = edge_candidate[0]
            else:
                edge = edge_candidate

            body = trajectory_obj.Body
            o_A_ol = mat_to_numpy(body.Placement.Matrix)
            print("Matrix used for transformation")
            displayMatrix(o_A_ol)

            # Determine number of points
            n_points = round(edge.Length * 1e-3 / trajectory_obj.DistanceBetweenPoints)
            if n_points < 2:
                n_points = 2

            if isinstance(edge.Curve, (Part.Line, Part.LineSegment)):
                start = edge.Vertexes[0].Point
                end = edge.Vertexes[-1].Point
                points_local = [start + (end - start)*(i/(n_points-1)) for i in range(n_points)]
            else:
                points_local = edge.discretize(Number=n_points)

            if sel.externalModel:
                # If externalModel, no transform?
                points_global = [(np.append(vec_to_numpy(pt), 1))[:3] for pt in points_local]
            else:
                # Apply body placement transform
                points_global = [(o_A_ol @ np.append(vec_to_numpy(pt), 1))[:3] for pt in points_local]

            print(f"Points on edge: {len(points_global)}")
            point_list.append(points_global)

    # Merge all lists of points
    merged_pts = chain_lists(point_list)
    trajectory_obj.Points = merged_pts
    return merged_pts


#
# --------------------- FUNCTIONALITY FOR SOLVING AND PLOTTING ---------------------
#
def solvePath():
    sel = FreeCADGui.Selection.getSelection()[0]
    robot = get_robot()

    # Prepare DH parameters
    DHraw = np.array(robot.DHPerameters, dtype=object)
    DHraw[:, 0] = 0.0
    DH = DHraw.astype(float)
    DH[:, 1:3] /= 1000
    target_dir = vec_to_numpy(robot.EndEffectorOrientation)

    # 1) Get raw 3D points (in meters)
    raw_pts = [vec_to_numpy(p)/1000.0 for p in computeTrajectoryPoints(sel)]
    if len(raw_pts) < 2:
        print("Not enough points!")
        return
    pts_np = np.array(raw_pts).T  # shape (3, N)

    # 2) Fit a parametric spline
    tck, u = splprep(pts_np, s=(sel.smoothing*1e-6))
    num_sample = len(raw_pts) * 4
    u_new = np.linspace(-sel.alpha, 1, num_sample)
    smooth_pts = np.array(splev(u_new, tck, ext=3)).T  # shape (M,3)

    from FreeCAD import Vector
    sel.SplinePoints = [Vector(*pt) for pt in smooth_pts]

    # 3) Arc-length to time
    arc = np.zeros(len(smooth_pts))
    for i in range(1, len(smooth_pts)):
        arc[i] = arc[i-1] + np.linalg.norm(smooth_pts[i] - smooth_pts[i-1])
    t_array = arc / sel.Velocity

    # 4) Derivatives
    x_dot = np.gradient(smooth_pts, t_array, axis=0)

    # 5) Inverse Kinematics
    q_list = []
    #angles_deg = []
    q = np.deg2rad(robot.Angles)
    for i, p in enumerate(smooth_pts):
        if i == 0:
            q = solve_ik(q, p, target_dir, DH)
        else:
            converged, q = inverse_kinematics_cpp.solveIK(q, p, target_dir, DH)
        q_list.append(q)
        #angles_deg.append(np.rad2deg(q).tolist())

    # 6) Joint velocities with Jacobian
    q_dot_list = []
    for i, q in enumerate(q_list):
        J = getJacobian(q)
        if J.ndim > 2:
            J = J[0]
        J_pos = J[0:3, :]
        q_dot = np.linalg.pinv(J_pos).dot(x_dot[i])
        q_dot_list.append(q_dot)
    q_dot_arr = np.array(q_dot_list)
    q_ddot_arr = np.gradient(q_dot_arr, t_array, axis=0)

    # 7) Store
    sel.q = q_list
    sel.t = t_array
    sel.q_dot = q_dot_arr
    sel.q_ddot = q_ddot_arr
    robot.Angles = np.rad2deg(q_list[-1]).tolist()

    print("Trajectory solved with parametric smoothing and pre-path extension.")
    displayMatrix(sel.t)


def updateTorques():
    robot = get_robot()
    M = np.array(robot.Masses[1:])
    InertiaMatrices = np.array([np.array(m) for m in robot.InertiaMatrices[1:]])
    CenterOfMass = np.array([np.array(m) for m in robot.CenterOfMass[1:]])
    DHperameters = np.array(robot.DHPerameters)
    DHperameters[:,0] = 0
    DHperameters = DHperameters.astype(float)
    DHperameters[:,1:3] /= 1000

    sel = FreeCADGui.Selection.getSelection()[0]
    q = sel.q
    q_dot = sel.q_dot
    q_ddot = sel.q_ddot

    tau = [compute_torque.computeJointTorques(
             qq, qd, qdd, M, InertiaMatrices, CenterOfMass, DHperameters
           ) for qq, qd, qdd in zip(q, q_dot, q_ddot)]
    sel.Torques = tau


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


class SolveTrajectoryCommand:
    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(os.path.dirname(__file__)), 'Resources', 'icons', 'solve.svg'),
            'MenuText': 'Solve Trajectory',
            'ToolTip': 'Solve inverse kinematics for chosen trajectory'
        }

    def Activated(self):
        profiler = cProfile.Profile()
        profiler.enable()

        sel = FreeCADGui.Selection.getSelection()
        if sel:
            trajectory_obj = sel[0]
            # Simply call the child's solve() method via the proxy:
            trajectory_obj.Proxy.solve()
            # Optionally call plotTrajectoryData():
            trajectory_obj.Proxy.plotTrajectoryData()

        profiler.disable()
        stats = pstats.Stats(profiler)
        stats.strip_dirs().sort_stats(pstats.SortKey.CUMULATIVE)
        sorted_stats = sorted(stats.stats.items(), key=lambda item: item[1][3], reverse=True)

        print("\nProfiling Results (Functions > 0.5 sec):")
        header_format = "{:<60s} {:>10s}"
        print(header_format.format("Function", "Time (sec)"))
        print("-" * 72)
        for func, stat in sorted_stats:
            cumulative_time = stat[3]
            if cumulative_time > 0.5:
                func_str = f"{func[2]} ({func[0]}:{func[1]})"
                print(header_format.format(func_str, f"{cumulative_time:.2f}"))

        print("\nProfiling completed.")

    def IsActive(self):
        sel = FreeCADGui.Selection.getSelection()
        if not sel:
            return False
        obj = sel[0]
        # Activate if the selected object has a `Type` property == 'Trajectory'
        return bool(obj and hasattr(obj, 'Type') and obj.Type == 'Trajectory')


FreeCADGui.addCommand('SolveTrajectoryCommand', SolveTrajectoryCommand())



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
