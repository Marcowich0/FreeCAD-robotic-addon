# Import necessary modules
import FreeCAD
import FreeCADGui
import Part
import os
from dynamics import computeJointTorques
import numpy as np
from PySide import QtCore
import threading

animation_state = "stopped"  # Possible states: "playing", "paused", "stopped"
current_animation_index = 0
animation_timer = None

from main_utils import currentSelectionType, get_robot, displayMatrix
from inverse_kinematics import solve_ik

# Define the custom object class with properties (corrected spelling)
class Trajectory:
    def __init__(self, obj):
        obj.Proxy = self
        
        # Add a property for linking to a Body (another FreeCAD object)
        obj.addProperty("App::PropertyString", "Type", "Trajectory", "Type of the object").Type = "Trajectory"
        obj.addProperty("App::PropertyLink", "Body", "Trajectory", "Link to a Body")
        obj.addProperty("App::PropertyLinkSubList", "Edges", "Trajectory", "Link to an Edges").Edges = []
        obj.addProperty("App::PropertyVectorList", "Points", "Trajectory", "List of points").Points = []
        obj.addProperty("App::PropertyFloat", "Velocity", "Trajectory", "Velocity").Velocity = 1

        obj.addProperty("App::PropertyPythonObject", "t", "Time", "List of times at each point").t = []
        obj.addProperty("App::PropertyPythonObject", "Angles", "Trajectory", "List of angles").Angles = []
        obj.addProperty("App::PropertyPythonObject", "q_dot", "Trajectory", "List of joint velocities").q_dot = []
        obj.addProperty("App::PropertyPythonObject", "q_ddot", "Trajectory", "List of joint accelerations").q_ddot = []
        obj.addProperty("App::PropertyPythonObject", "Torques", "Trajectory", "List of joint torques").Torques = []

        obj.addProperty("App::PropertyFloat", "smoothing", "Trajectory", "Smoothing factor").smoothing = 1
        obj.addProperty("App::PropertyFloat", "alpha", "Trajectory", "Pre-path extension").alpha = 0.1
        
        obj.addProperty("App::PropertyFloat", "DistanceBetweenPoints", "Trajectory", "Distance between points").DistanceBetweenPoints = 5e-3
    
    def execute(self, obj):
        pass  # Recompute logic here



# View provider with corrected name and added icon
class ViewProviderTrajectory:
    def __init__(self, vobj):
        vobj.Proxy = self

    def attach(self, vobj):
        self.Object = vobj.Object
        return

    def getIcon(self):  # Added icon method
        return ":/icons/Trajectory.svg"

    # Remaining view provider methods
    def updateData(self, fp, prop):
        return

    def getDisplayModes(self, obj):
        return []

    def getDefaultDisplayMode(self):
        return "Shaded"

    def setDisplayMode(self, mode):
        return mode

    def onChanged(self, vp, prop):
        return

    def __getstate__(self):
        return {}

    def __setstate__(self, state):
        return None

# ---------------------------------------------------------------------------
# Command to add a Trajectory object.
class AddTrajectoryCommand:
    def GetResources(self):
        return {'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'trajectory.svg'), 
                'MenuText': 'Add Trajectory', 
                'ToolTip': 'Add a trajectory object'}

    def Activated(self):
        doc = FreeCAD.activeDocument()
        trajectory_obj = doc.addObject("App::FeaturePython", "Trajectory")
        doc.Assembly.addObject(trajectory_obj)
            
        Trajectory(trajectory_obj)
        trajectory_obj.ViewObject.Proxy = ViewProviderTrajectory(trajectory_obj.ViewObject)

        # Instead of calculating points now, only store the edge (and parent body)
        selectEdgesForTrajectory(trajectory_obj)

        doc.recompute()

    def IsActive(self):
        return bool(currentSelectionType() == 'Edge' and get_robot())

FreeCADGui.addCommand('AddTrajectoryCommand', AddTrajectoryCommand())

# ---------------------------------------------------------------------------
# New helper function: only stores the edge (and body) on the trajectory object.
def selectEdgesForTrajectory(trajectory_obj):
    collected_edges = []

    for sel in FreeCADGui.Selection.getSelectionEx():
        for subel_name in sel.SubElementNames:
            collected_edges.append((sel.Object, subel_name))

    # Assign them directly
    trajectory_obj.Edges = collected_edges

    # If you still want to set Body, we look at the first entry's parents
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
    #arrs = [[np.array((v.x, v.y, v.z), float) for v in lst] for lst in list_of_lists]
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
            if d1 < best_dist: best_dist, best_idx, best_flip, best_prepend = d1, i, False, False
            d2 = np.linalg.norm(ce - se)  # append reversed
            if d2 < best_dist: best_dist, best_idx, best_flip, best_prepend = d2, i, True, False
            d3 = np.linalg.norm(cs - ss)  # prepend reversed
            if d3 < best_dist: best_dist, best_idx, best_flip, best_prepend = d3, i, True, True
            d4 = np.linalg.norm(cs - se)  # prepend normal
            if d4 < best_dist: best_dist, best_idx, best_flip, best_prepend = d4, i, False, True
        chosen = arrs.pop(best_idx)
        if best_flip:
            chosen.reverse()
        if best_prepend:
            chain = chosen[:-1] + chain
        else:
            chain = chain + chosen[1:]
    return [FreeCAD.Vector(*p) for p in chain]




# ---------------------------------------------------------------------------
# New helper function: compute trajectory points from the stored edge.
def computeTrajectoryPoints(trajectory_obj):
    if not trajectory_obj.Edges:
         print("Error: No edge stored in the trajectory object!")
         return []
    
    # Retrieve the stored reference (object and subelement name)
    point_list = []
    for obj, sub in trajectory_obj.Edges:
        for subelement in sub:
            print("edge array")
            print(obj, subelement)
            edge_candidate = obj.getSubObject(subelement)
            
            # If getSubObject returns a tuple, take the first element.
            if isinstance(edge_candidate, tuple):
                edge = edge_candidate[0]
            else:
                edge = edge_candidate
                
            from main_utils import mat_to_numpy
            body = trajectory_obj.Body
            o_A_ol = mat_to_numpy(body.Placement.Matrix)
            print("Matrix used for transformation")
            displayMatrix(o_A_ol)
            
            # Determine the number of points based on the edge length
            n_points = round(edge.Length * 1e-3 / trajectory_obj.DistanceBetweenPoints)
            if n_points < 2:
                n_points = 2  # Ensure at least two points

            # Discretize based on the edge type
            if isinstance(edge.Curve, (Part.Line, Part.LineSegment)):
                start = edge.Vertexes[0].Point
                end = edge.Vertexes[-1].Point
                points_local = [start + (end - start) * i/(n_points-1) for i in range(n_points)]
            else:
                points_local = edge.discretize(Number=n_points)

            # Transform points to global coordinates using the body's placement
            points_global = [(np.append(vec_to_numpy(point),1))[:3] for point in points_local]
            print(f"Points on edge: {len(points_global)}")
            point_list.append(points_global)

    points = chain_lists(point_list)
    trajectory_obj.Points = points
    return points

import cProfile
import pstats
import threading
# ---------------------------------------------------------------------------
# Command to solve the trajectory (compute points and then inverse kinematics)
class SolveTrajectoryCommand:
    def GetResources(self):
        return {'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'solve.svg'), 
                'MenuText': 'Solve Trajectory', 
                'ToolTip': 'Solve inverse kinematics for chosen trajectory'}



    def Activated(self):
        profiler = cProfile.Profile()
        profiler.enable()

        solvePath()
        updateTorques()
        plotTrajectoryData()



        profiler.disable()

        # Print profiling results to console (simplified)
        stats = pstats.Stats(profiler)
        stats.strip_dirs().sort_stats(pstats.SortKey.CUMULATIVE)
        
        # Explicitly sort the stats by cumulative time (stat index 3) in descending order.
        sorted_stats = sorted(stats.stats.items(), key=lambda item: item[1][3], reverse=True)
        
        print("\nProfiling Results (Functions > 0.5 sec):")
        
        # Define a header format: left-align the function name and right-align the time.
        header_format = "{:<60s} {:>10s}"
        print(header_format.format("Function", "Time (sec)"))
        print("-" * 72)
        
        # Each key in stats.stats is a tuple: (filename, line number, function name)
        # stat[3] is the cumulative time.
        for func, stat in sorted_stats:
            cumulative_time = stat[3]
            if cumulative_time > 0.5:
                func_str = f"{func[2]} ({func[0]}:{func[1]})"
                print(header_format.format(func_str, f"{cumulative_time:.2f}"))
        
        print("\nProfiling completed.")

    def IsActive(self):
        sel = FreeCADGui.Selection.getSelection()[0]
        return bool(sel and hasattr(sel, 'Type') and sel.Type == 'Trajectory')
    

FreeCADGui.addCommand('SolveTrajectoryCommand', SolveTrajectoryCommand())

# ---------------------------------------------------------------------------
# Command to play the trajectory animation.
class PlayTrajectoryCommand:
    def GetResources(self):
        return {'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'play.svg'),
                'MenuText': 'Play Trajectory',
                'ToolTip': 'Start/resume trajectory animation'}

    def Activated(self):
        global animation_state, current_animation_index, animation_timer
        
        if animation_state == "playing":
            return

        sel = FreeCADGui.Selection.getSelection()[0]
        if not sel.Angles:
            print("No angles calculated!")
            return

        # Initialize timer if it does not exist
        if not animation_timer:
            animation_timer = QtCore.QTimer()
            animation_timer.timeout.connect(self.update_animation)

        # Start or resume animation
        if animation_state == "stopped":
            current_animation_index = 0
            self.update_robot_position(sel.Angles[0])

        animation_state = "playing"
        # Use DistanceBetweenPoints and Velocity to compute the delay in ms.
        delay = int((sel.DistanceBetweenPoints / sel.Velocity) * 1000)
        animation_timer.start(delay)

    def update_animation(self):
        global animation_state, current_animation_index
        
        sel = FreeCADGui.Selection.getSelection()[0]
        if not sel or current_animation_index >= len(sel.Angles):
            self.stop_animation()
            return

        self.update_robot_position(sel.Angles[current_animation_index])
        current_animation_index += 1

    def update_robot_position(self, angles):
        robot = get_robot()
        if robot:
            robot.Angles = angles
            FreeCADGui.updateGui()

    def stop_animation(self):
        global animation_state, animation_timer
        if animation_timer:
            animation_timer.stop()
        animation_state = "stopped"

    def IsActive(self):
        sel = FreeCADGui.Selection.getSelection()[0] if FreeCADGui.Selection.getSelection() else None
        return bool(sel and hasattr(sel, 'Type') and sel.Type == 'Trajectory' 
                   and animation_state != "playing" and sel.Angles)


class PauseTrajectoryCommand:
    def GetResources(self):
        return {'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'pause.svg'),
                'MenuText': 'Pause Trajectory',
                'ToolTip': 'Pause trajectory animation'}

    def Activated(self):
        global animation_state, animation_timer
        if animation_state == "playing":
            animation_state = "paused"
            if animation_timer:
                animation_timer.stop()

    def IsActive(self):
        return animation_state == "playing"

class StopTrajectoryCommand:
    def GetResources(self):
        return {'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'stop.svg'),
                'MenuText': 'Stop Trajectory',
                'ToolTip': 'Stop and reset trajectory animation'}

    def Activated(self):
        global animation_state, current_animation_index
        animation_state = "stopped"
        current_animation_index = 0
        
        # Reset to initial position
        sel = FreeCADGui.Selection.getSelection()[0]
        if sel.Angles:
            get_robot().Angles = sel.Angles[0]
            FreeCADGui.updateGui()
        
        if animation_timer:
            animation_timer.stop()

    def IsActive(self):
        return animation_state in ["playing", "paused"]

# Register animation commands
FreeCADGui.addCommand('PlayTrajectoryCommand', PlayTrajectoryCommand())
FreeCADGui.addCommand('PauseTrajectoryCommand', PauseTrajectoryCommand())
FreeCADGui.addCommand('StopTrajectoryCommand', StopTrajectoryCommand())

import inverse_kinematics_cpp
from main_utils import vec_to_numpy

from scipy.interpolate import CubicSpline

from scipy.interpolate import splprep, splev
from forward_kinematics import getJacobian

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

    # 2) Fit a parametric spline to the raw points
    # 'smoothing' controls rounding; 'alpha' extends the path before the computed path

    tck, u = splprep(pts_np, s=(sel.smoothing*1e-6))
    num_sample = len(raw_pts) * 4
    u_new = np.linspace(-sel.alpha, 1, num_sample)
    smooth_pts = np.array(splev(u_new, tck, ext=3)).T  # shape (M,3)

    # 3) Reparameterize by arc-length to compute time stamps (assuming constant speed)
    arc = np.zeros(len(smooth_pts))
    for i in range(1, len(smooth_pts)):
        arc[i] = arc[i-1] + np.linalg.norm(smooth_pts[i] - smooth_pts[i-1])
    t_array = arc / sel.Velocity  # time = arc length / speed

    # 4) Compute first and second derivatives via finite differences
    x_dot = np.gradient(smooth_pts, t_array, axis=0)

    # 5) Solve inverse kinematics along the smooth path
    q_list = []
    angles_deg = []
    q = np.deg2rad(robot.Angles)
    for i, p in enumerate(smooth_pts):
        if i == 0:
            q = solve_ik(q, p, target_dir, DH)
        else:
            converged, q = inverse_kinematics_cpp.solveIK(q, p, target_dir, DH)
        q_list.append(q)
        angles_deg.append(np.rad2deg(q).tolist())

    # 6) Compute joint velocities using the Jacobian from forward_kinematics
    q_dot_list = []
    for i, q in enumerate(q_list):
        J = getJacobian(q)
        if J.ndim > 2:
            J = J[0]
        J_pos = J[0:3, :]  # take only the position rows
        q_dot = np.linalg.pinv(J_pos).dot(x_dot[i])
        q_dot_list.append(q_dot)
    q_dot_arr = np.array(q_dot_list)
    q_ddot_arr = np.gradient(q_dot_arr, t_array, axis=0)

    # 7) Store results in the trajectory object and update the robot
    sel.Angles = angles_deg
    sel.t = t_array
    sel.q_dot = q_dot_arr
    sel.q_ddot = q_ddot_arr
    robot.Angles = angles_deg[-1]

    print("Trajectory solved with parametric smoothing and pre-path extension.")
    displayMatrix(sel.t)



def updateTorques():
    robot = get_robot()
    M = np.array(robot.Masses[1:])
    InertiaMatrices = np.array(robot.InertiaMatrices[1:])
    CenterOfMass = np.array(robot.CenterOfMass[1:])
    DHperameters = np.array(robot.DHPerameters)
    DHperameters[:,0] = 0
    DHperameters = DHperameters.astype(float)
    DHperameters[:,1:3] /= 1000


    sel = FreeCADGui.Selection.getSelection()[0]
    q = np.deg2rad(sel.Angles)
    q_dot = sel.q_dot
    q_ddot = sel.q_ddot

    tau = [compute_torque.computeJointTorques(q, q_dot, q_ddot, M, InertiaMatrices, CenterOfMass, DHperameters) for q, q_dot, q_ddot in zip(q, q_dot, q_ddot)]
    sel.Torques = tau

def plotTrajectoryData():
    import matplotlib.pyplot as plt
    import numpy as np
    from FreeCADGui import Selection

    # Use dark background style as a base
    plt.style.use('dark_background')
    dark_gray = '#303030'
    plt.rcParams['axes.facecolor'] = dark_gray
    plt.rcParams['figure.facecolor'] = dark_gray

    # Helper function: insert NaNs where discontinuities occur to break the line
    def insert_nan_breaks(x, y, threshold=np.pi):
        new_x = [x[0]]
        new_y = [y[0]]
        for i in range(1, len(x)):
            if np.abs(y[i] - y[i - 1]) > threshold:
                new_x.append(np.nan)
                new_y.append(np.nan)
            new_x.append(x[i])
            new_y.append(y[i])
        return np.array(new_x), np.array(new_y)

    # Get the current selected trajectory object
    sel = Selection.getSelection()[0]

    # Convert lists to numpy arrays
    t = np.array(sel.t)
    # Convert angles from degrees to radians and wrap into [-pi, pi]
    angles = np.deg2rad(sel.Angles)
    q_dot = np.array(sel.q_dot)
    q_ddot = np.array(sel.q_ddot)
    torques = np.array(sel.Torques)

    # Ensure that angles, velocities, and accelerations are 2D arrays (each column represents a joint)
    if angles.ndim == 1:
        angles = angles.reshape(-1, 1)
        q_dot = q_dot.reshape(-1, 1)
        q_ddot = q_ddot.reshape(-1, 1)
    n_joints = angles.shape[1]

    # Create a figure with 4 vertically-stacked subplots sharing the time axis
    fig, axs = plt.subplots(4, 1, sharex=True, figsize=(8, 10))

    # --- Joint Positions ---
    for j in range(n_joints):
        # Wrap the joint angles to [-pi, pi]
        wrapped = (angles[:, j] + np.pi) % (2 * np.pi) - np.pi
        # Insert NaNs at discontinuities to avoid vertical lines
        new_t, new_wrapped = insert_nan_breaks(t, wrapped, threshold=np.pi)
        axs[0].plot(new_t, new_wrapped, label=f'Joint {j+1}')
    axs[0].set_ylabel('Position (rad)')
    axs[0].set_title('Joint Positions')
    axs[0].grid(True, color='gray')
    axs[0].set_ylim(-np.pi, np.pi)
    ticks = [-np.pi, -np.pi/2, 0, np.pi/2, np.pi]
    labels = [r'$-\pi$', r'$-\pi/2$', '0', r'$\pi/2$', r'$\pi$']
    axs[0].set_yticks(ticks)
    axs[0].set_yticklabels(labels)
    axs[0].legend(loc='upper right')

    # --- Joint Velocities ---
    for j in range(n_joints):
        axs[1].plot(t, q_dot[:, j], label=f'Joint {j+1}')
    axs[1].set_ylabel('Velocity (rad/s)')
    axs[1].set_title('Joint Velocities')
    axs[1].grid(True, color='gray')
    axs[1].legend(loc='upper right')

    # --- Joint Accelerations ---
    for j in range(n_joints):
        axs[2].plot(t, q_ddot[:, j], label=f'Joint {j+1}')
    axs[2].set_ylabel('Acceleration (rad/s²)')
    axs[2].set_title('Joint Accelerations')
    axs[2].grid(True, color='gray')
    axs[2].legend(loc='upper right')

    # --- Joint Torques ---
    if torques.ndim == 1:
        torques = torques.reshape(-1, 1)
    for j in range(torques.shape[1]):
        axs[3].plot(t, torques[:, j], label=f'Joint {j+1}')
    axs[3].set_ylabel('Torque (Nm)')
    axs[3].set_xlabel('Time (s)')
    axs[3].set_title('Joint Torques')
    axs[3].grid(True, color='gray')
    axs[3].legend(loc='upper right')

    plt.tight_layout()
    plt.show()

import compute_torque

class AddTrajectoryCommand2:
    def GetResources(self):
        return {'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'trajectory.svg'), 
                'MenuText': 'Add Trajectory', 
                'ToolTip': 'Add a trajectory object'}

    def Activated(self):
        print("testing")
        robot = get_robot()
        q = np.deg2rad(get_robot().Angles)
        q_dot = np.zeros_like(q)
        q_ddot = np.zeros_like(q)
        M = np.array(robot.Masses[1:])
        InertiaMatrices = np.array(robot.InertiaMatrices[1:])
        CenterOfMass = np.array(robot.CenterOfMass[1:])
        DHperameters = np.array(robot.DHPerameters)
        DHperameters[:,0] = 0
        DHperameters = DHperameters.astype(float)
        DHperameters[:,1:3] /= 1000


        q_ddot[0] = 1
        torques = compute_torque.computeJointTorques(q, q_dot, q_ddot, M, InertiaMatrices, CenterOfMass, DHperameters)
        D, C, g = compute_torque.getMatrices(q, np.ones_like(q), M, InertiaMatrices, CenterOfMass, DHperameters)
        print("Computed D matrix [C++]:")
        displayMatrix(D)
        print("Computed C matrix [C++]:")
        displayMatrix(C)
        print("Computed gravity [C++]:")
        displayMatrix(g)
        print("Computed torques [C++]:")
        displayMatrix(torques)

        
        tau = computeJointTorques(q, q_dot, q_ddot)
        print(f"Torques [Python]:")
        displayMatrix(tau)




    def IsActive(self):
        return get_robot() is not None

FreeCADGui.addCommand('AddTrajectoryCommand2', AddTrajectoryCommand2())