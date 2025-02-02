# Import necessary modules
import FreeCAD
import FreeCADGui
import Part
import time

from main_utils import currentSelectionType, get_robot
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
        obj.addProperty("App::PropertyFloat", "Velocity", "Trajectory", "Velocity").Velocity = 1
        obj.addProperty("App::PropertyPythonObject", "Angles", "Trajectory", "List of angles").Angles = []
    
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



class AddTrajectoryCommand:
    def GetResources(self):
        return {'Pixmap': 'path/to/icon', 
                'MenuText': 'Add Trajectory', 
                'ToolTip': 'Add a trajectory object'}

    def Activated(self):
        doc = FreeCAD.activeDocument()
        trajectory_obj = doc.addObject("App::FeaturePython", "Trajectory")
        doc.Assembly.addObject(trajectory_obj)
            
        Trajectory(trajectory_obj)
        trajectory_obj.ViewObject.Proxy = ViewProviderTrajectory(trajectory_obj.ViewObject)

        selectLinePoints(trajectory_obj)

        doc.recompute()

    def IsActive(self):
        return bool(currentSelectionType() == 'Edge' and get_robot())

FreeCADGui.addCommand('AddTrajectoryCommand', AddTrajectoryCommand())



class SolveTrajectoryCommand:
    def GetResources(self):
        return {'Pixmap': 'path/to/icon', 
                'MenuText': 'Add Trajectory', 
                'ToolTip': 'Add a trajectory object'}

    def Activated(self):
        robot = get_robot()
        sel = FreeCADGui.Selection.getSelection()[0]
        angles = []
        for point in sel.Points:
            solve_ik(point)
            angles.append(robot.Angles)
            
        sel.Angles = angles

    def IsActive(self):
        sel = FreeCADGui.Selection.getSelection()[0]
        return bool(sel and hasattr(sel, 'Type') and sel.Type == 'Trajectory')

FreeCADGui.addCommand('SolveTrajectoryCommand', SolveTrajectoryCommand())



class AnimateTrajectoryCommand:
    def GetResources(self):
        return {'Pixmap': 'path/to/icon', 
                'MenuText': 'Add Trajectory', 
                'ToolTip': 'Add a trajectory object'}

    def Activated(self):
        robot = get_robot()
        sel = FreeCADGui.Selection.getSelection()[0]
        for angles in sel.Angles:
            delay = 1 / sel.Velocity
            robot.Angles = angles
            FreeCADGui.updateGui()
            time.sleep(delay)

    def IsActive(self):
        sel = FreeCADGui.Selection.getSelection()[0]
        return bool(sel and hasattr(sel, 'Type') and sel.Type == 'Trajectory')

FreeCADGui.addCommand('AnimateTrajectoryCommand', AnimateTrajectoryCommand())




def selectLinePoints(trajectory_obj):
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

    # Find parent body in assembly
    for parent in obj.Parents:
        if parent[0].Name == 'Assembly':
            body_name = parent[1].split('.')[0]
            trajectory_obj.Body = FreeCAD.ActiveDocument.getObject(body_name)
            o_A_ol = trajectory_obj.Body.Placement.Matrix

    # Calculate points
    n_points = round(edge.Length)
    if isinstance(edge.Curve, (Part.Line, Part.LineSegment)):
        start = edge.Vertexes[0].Point
        end = edge.Vertexes[-1].Point
        points_local = [start + (end - start) * i/(n_points-1) for i in range(n_points)]
    else:
        points_local = edge.discretize(Number=n_points)

    # Transform points to global coordinates
    points_global = [o_A_ol.multVec(point) for point in points_local]
    trajectory_obj.Points = points_global

    print(f"Generated {len(points_global)} trajectory points:")