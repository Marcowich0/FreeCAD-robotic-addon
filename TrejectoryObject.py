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
        obj.addProperty("App::PropertyLinkSub", "Edge", "Trajectory", "Link to an Edge")
        obj.addProperty("App::PropertyVectorList", "Points", "Trajectory", "List of points").Points = []
        obj.addProperty("App::PropertyFloat", "Velocity", "Trajectory", "Velocity").Velocity = 100

        obj.addProperty("App::PropertyPythonObject", "t", "Time", "List of times at each point").t = []
        obj.addProperty("App::PropertyPythonObject", "Angles", "Trajectory", "List of angles").Angles = []
        obj.addProperty("App::PropertyPythonObject", "q_dot", "Trajectory", "List of joint velocities").q_dot = []
        obj.addProperty("App::PropertyPythonObject", "q_ddot", "Trajectory", "List of joint accelerations").q_ddot = []
        obj.addProperty("App::PropertyPythonObject", "Torques", "Trajectory", "List of joint torques").Torques = []
        

        obj.addProperty("App::PropertyInteger", "NumberOfPoints", "Trajectory", "Number of points").NumberOfPoints = 100
        obj.addProperty("App::PropertyFloat", "DistanceBetweenPoints", "Trajectory", "Distance between points").DistanceBetweenPoints = 0.0
    
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
        selectEdgeForTrajectory(trajectory_obj)

        doc.recompute()

    def IsActive(self):
        return bool(currentSelectionType() == 'Edge' and get_robot())

FreeCADGui.addCommand('AddTrajectoryCommand', AddTrajectoryCommand())

# ---------------------------------------------------------------------------
# New helper function: only stores the edge (and body) on the trajectory object.
def selectEdgeForTrajectory(trajectory_obj):
    selection = FreeCADGui.Selection.getSelectionEx()
    
    if not selection or not selection[0].SubElementNames:
        print("Error: No selection or invalid selection.")
        return

    sel = selection[0]
    subelement = sel.SubElementNames[0]
    
    if not subelement.startswith('Edge'):
        print("Error: Please select an edge.")
        return

    obj = sel.Object
    edge = obj.getSubObject(subelement)
    
    if not edge:
        print("Error: Could not retrieve the selected edge.")
        return

    # Store the edge reference using PropertyLinkSub
    trajectory_obj.Edge = (obj, subelement)

    # Find parent body in assembly and store it (if applicable)
    for parent in obj.Parents:
        if parent[0].Name == 'Assembly':
            body_name = parent[1].split('.')[0]
            trajectory_obj.Body = FreeCAD.ActiveDocument.getObject(body_name)
            break

    print("Edge selected for trajectory. Points will be computed during solve.")

# ---------------------------------------------------------------------------
# New helper function: compute trajectory points from the stored edge.
def computeTrajectoryPoints(trajectory_obj):
    if not trajectory_obj.Edge:
         print("Error: No edge stored in the trajectory object!")
         return []
    
    # Retrieve the stored reference (object and subelement name)
    obj, subelement = trajectory_obj.Edge
    edge_candidate = obj.getSubObject(subelement)
    
    # If getSubObject returns a tuple, take the first element.
    if isinstance(edge_candidate, tuple):
         edge = edge_candidate[0]
    else:
         edge = edge_candidate
         
    if not edge:
         print("Error: Could not retrieve edge from stored reference.")
         return []
    
    # Use the stored Body (if available) for the transformation; otherwise assume identity
    if trajectory_obj.Body:
         body = trajectory_obj.Body
         o_A_ol = body.Placement.Matrix
    else:
         o_A_ol = FreeCAD.Matrix()
    
    # Determine the number of points based on the edge length
    n_points = trajectory_obj.NumberOfPoints
    trajectory_obj.DistanceBetweenPoints = edge.Length/n_points
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
    points_global = [o_A_ol.multVec(point) for point in points_local]
    trajectory_obj.Points = points_global

    print(f"Computed {len(points_global)} trajectory points.")
    return points_global

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
        solveDynamics()



        profiler.disable()

        # Print profiling results to console (simplified)
        stats = pstats.Stats(profiler)
        stats.strip_dirs().sort_stats(pstats.SortKey.CUMULATIVE)
        
        # Explicitly sort the stats by cumulative time (stat index 3) in descending order.
        sorted_stats = sorted(stats.stats.items(), key=lambda item: item[1][3], reverse=True)
        
        print("\nProfiling Results (Functions > 2 sec):")
        
        # Define a header format: left-align the function name and right-align the time.
        header_format = "{:<60s} {:>10s}"
        print(header_format.format("Function", "Time (sec)"))
        print("-" * 72)
        
        # Each key in stats.stats is a tuple: (filename, line number, function name)
        # stat[3] is the cumulative time.
        for func, stat in sorted_stats:
            cumulative_time = stat[3]
            if cumulative_time > 1.0:
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



def solvePath():
    sel = FreeCADGui.Selection.getSelection()[0]
    
    # Compute the trajectory points from the stored edge
    points = computeTrajectoryPoints(sel)
    if not points:
        print("No trajectory points computed!")
        return
    angles = []
    for i, point in enumerate(points):
        sol = solve_ik(point, collision=False) if i>0 else solve_ik(point, collision=True) 
        angles.append(sol)
    sel.Angles = angles
        
    time = [0, *[sel.DistanceBetweenPoints/sel.Velocity for _ in range(len(sel.Angles)-1)]]
    sel.t = np.cumsum(time)
    print("Trajectory solved and angles stored.")


def solveDynamics():
    updateVelocityAndAcceleration()
    updateTorques()


def updateVelocityAndAcceleration():
    sel = FreeCADGui.Selection.getSelection()[0]
    if not sel.Angles:
        print("No angles calculated!")
        return
    
    q = sel.Angles
    q_dot = np.gradient(q, sel.t, axis=0)
    q_ddot = np.gradient(q_dot, sel.t, axis=0)
    sel.q_dot = q_dot
    sel.q_ddot = q_ddot
    print("Velocity and acceleration updated.")


def updateTorques():
    robot = get_robot()
    sel = FreeCADGui.Selection.getSelection()[0]
    q = np.deg2rad(sel.Angles)
    q_dot = sel.q_dot
    q_ddot = sel.q_ddot
    #tau = [computeJointTorques(q, q_dot, q_ddot) for q, q_dot, q_ddot in zip(q, q_dot, q_ddot)]
    M = np.array(robot.Masses[1:])
    InertiaMatrices = np.array(robot.InertiaMatrices[1:])
    CenterOfMass = np.array(robot.CenterOfMass[1:])
    DHperameters = np.array(robot.DHPerameters)
    DHperameters[:,0] = 0
    DHperameters = DHperameters.astype(float)
    DHperameters[:,1:3] /= 1000
    tau = [compute_torque.computeJointTorques(q, q_dot, q_ddot, M, InertiaMatrices, CenterOfMass, DHperameters) for q, q_dot, q_ddot in zip(q, q_dot, q_ddot)]
    sel.Torques = tau


def plotTorques():
    import matplotlib.pyplot as plt
    sel = FreeCADGui.Selection.getSelection()[0]

    # Use dark background style as a base
    plt.style.use('dark_background')

    # Override the facecolors to a dark gray
    dark_gray = '#303030'
    plt.rcParams['axes.facecolor'] = dark_gray
    plt.rcParams['figure.facecolor'] = dark_gray

    plt.plot(sel.t, sel.Torques)
    plt.grid(True, color='gray')  # Ensure grid lines are visible
    plt.legend([f"Joint {i+1}" for i in range(len(sel.Torques[0]))])
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (Nm)")
    plt.show()

import sys
sys.path.append(r"D:\FreeCAD\Mod\FreeCAD-robotic-addon\robot_dynamics_module\build\Release")

from forward_kinematics import getDHTransformations, getJacobianCenter

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
        displayMatrix(DHperameters)


        q_ddot[0] = 1
        torques = compute_torque.computeJointTorques(q, q_dot, q_ddot, M, InertiaMatrices, CenterOfMass, DHperameters)
        D, C, g = compute_torque.getMatrices(q, q_dot, M, InertiaMatrices, CenterOfMass, DHperameters)
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
