import FreeCAD
import FreeCADGui
import os
import re

class RobotObject:
    def __init__(self, obj):
        # Link the custom object to this proxy
        obj.Proxy = self
        
        obj.addProperty("App::PropertyLinkList", "Constraints", "Robot", "List of links").Constraints = []
        obj.addProperty("App::PropertyLinkList", "Bodies", "Robot", "List of bodies").Bodies = []
        obj.addProperty("App::PropertyIntegerList", "Edges", "Robot", "List of edges").Edges = []
        obj.addProperty("App::PropertyLinkList", "CoordinateSystems", "Robot", "List of coordinate systems").CoordinateSystems = []
        obj.addProperty("App::PropertyLinkList", "BodyCoordinateSystems", "Robot", "List of body coordinate systems").BodyCoordinateSystems = []

        obj.addProperty("App::PropertyLink", "Base", "Robot", "Base of the robot").Base = None

        obj.addProperty("App::PropertyString", "Type", "Base", "Type of the object").Type = "Robot"
        obj.setEditorMode("Type", 1)  # Make the property read-only
        #obj.setEditorMode("Bodies", 1)  # Make the property read-only
        
        # Additional setup if needed
        self.Type = "Robot"

    def execute(self, obj):
        """Define how the object behaves when updated."""
        FreeCAD.Console.PrintMessage("Executing RobotObject\n")
    
    def onChanged(self, obj, prop):
        """Handle property changes."""
        if prop == "Constraints":
            FreeCAD.Console.PrintMessage(f"Constraints updated: {obj.Constraints}\n")



class RobotViewProvider:
    def __init__(self, obj):
        obj.Proxy = self

    def attach(self, obj):
        """Called when attached to an object."""
        self.Object = obj

    def getIcon(self):
        """Define a custom icon."""
        return ":/icons/Robot.svg"  # Change this to an actual icon path if you have one

    def updateData(self, fp, prop):
        """Called when the data of the object is updated."""
        pass

    def setEdit(self, vobj, mode):
        """Called when entering edit mode."""
        return False

    def unsetEdit(self, vobj, mode):
        """Called when leaving edit mode."""
        return False

    def doubleClicked(self, vobj):
        """Handle double-clicking on the object in the Tree View."""
        FreeCAD.Console.PrintMessage("Robot object was double-clicked\n")
        return True
    

def initialize_robot():
    """Directly initialize the robot object in the active document."""
    doc = FreeCAD.ActiveDocument
    if doc is None:
        doc = FreeCAD.newDocument()

    # Create a group to contain the robot
    robot_group = doc.addObject("App::DocumentObjectGroup", "RobotContainer")
    robot_group.Label = "Robot Container"

    # Create the Robot object
    robot_obj = doc.addObject("App::FeaturePython", "Robot")
    RobotObject(robot_obj)  # Initialize the custom robot object
    RobotViewProvider(robot_obj.ViewObject)  # Attach the view provider

    # Add the Robot object to the group
    robot_group.addObject(robot_obj)

    doc.recompute()
    return robot_obj



# ----------------- Adding the GUI Button -----------------
class CreateRobotCommand:
    """A FreeCAD command to create a Robot object."""

    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'robotArm.svg'),
            'MenuText': 'Create Robot',
            'ToolTip': 'Instantiate a new Robot'
        }

    def Activated(self):
        """Called when the command is activated (button clicked)."""
        initialize_robot()

    def IsActive(self):
        """Determine if the command should be active."""
        if get_robot() is None:
            return True
        return False
    


def get_robot():
    for obj in FreeCAD.ActiveDocument.Objects:
        if hasattr(obj, 'Type') and obj.Type == 'Robot':
            return obj
    return None


def drawDanevitHartenberg(obj):
    if not obj.CoordinateSystems:
        """Draw the robot using the Denavit-Hartenberg parameters."""
        lcs = FreeCAD.ActiveDocument.addObject( 'PartDesign::CoordinateSystem', f'LCS_link_{0}' )
        obj.Base.LinkedObject.addObject(lcs)
    else:
        lcs = obj.CoordinateSystems[0]

    lcs.Placement.Rotation = FreeCAD.Rotation(FreeCAD.Vector(1,0,0), FreeCAD.Vector(0,1,0), FreeCAD.Vector(0,0,1))
    lcs_arr = [lcs]

        
    for i, joint, body, edge in zip(range(len(obj.Constraints)), obj.Constraints, obj.Bodies, obj.Edges):
        if not obj.CoordinateSystems:
            lcs = FreeCAD.ActiveDocument.addObject( 'PartDesign::CoordinateSystem', f'LCS_link_{i+1}' ) # Adds coordinate system to the document
        else:
            lcs = obj.CoordinateSystems[i+1]

        #edge_nr = int(re.search(r'\d+$', joint.Reference1[1][0]).group()) - 1 # Finds edge number from reference
        edge_nr = edge

        print(f"ref body: {body.Name}, ref edge nr {edge_nr}, joint name: {body.Name}") # debug

        circle = FreeCAD.ActiveDocument.getObject(body.Name).Shape.Edges[edge_nr].Curve # Finds the circle of the constraint
        lcs.Placement.Base = circle.Center # Sets the base of the coordinate system to the center of the circle
        
        last_x = lcs_arr[-1].Placement.Rotation.multVec(FreeCAD.Vector(1,0,0))
        last_z = lcs_arr[-1].Placement.Rotation.multVec(FreeCAD.Vector(0,0,1))
        
        parallel = last_z.cross(circle.Axis).Length < 1e-6 # Checks if the circle axis is parallel to the last z-axis
        print(f"parallel: {parallel}") # debug
        if parallel:
            z_axis = last_z
            x_axis = last_x

        else:
            z_axis = circle.Axis # Sets the axis of the coordinate system to the axis of the circle
            x_axis = last_z.cross(z_axis)

        y_axis = z_axis.cross(x_axis)

        lcs.Placement.Rotation = FreeCAD.Rotation(x_axis, y_axis, z_axis)
        obj.Base.LinkedObject.addObject(lcs)
        lcs_arr.append(lcs)

        datum_plane = FreeCAD.ActiveDocument.addObject('PartDesign::Plane', f'plane_on_DH_coordinates_{i+1}')
        obj.Base.LinkedObject.addObject(datum_plane)
        datum_plane.AttachmentOffset = FreeCAD.Placement(
            FreeCAD.Vector(0.0, 0.0, 0.0),  # Base position of the plane
            FreeCAD.Rotation(FreeCAD.Vector(0, 1, 0), 0.0)  # No additional rotation
        )
        datum_plane.MapReversed = False
        datum_plane.AttachmentSupport = [(lcs, '')]  # Attach to the LCS
        datum_plane.MapPathParameter = 0.0
        datum_plane.MapMode = 'ObjectXZ'  # Align to the X-Z plane of the LCS
        datum_plane.recompute()

        print(f"z_axis: {z_axis}, x_axis: {x_axis}, y_axis: {y_axis}") # debug
        print(f"length of z_axis: {round(z_axis.Length, 3)}, length of x_axis: {round(x_axis.Length, 3)}, length of y_axis: {round(y_axis.Length, 3)}")
        print(" -- ")
        

    obj.CoordinateSystems = lcs_arr

FreeCADGui.addCommand('CreateRobotCommand', CreateRobotCommand())