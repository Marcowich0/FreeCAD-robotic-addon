import FreeCAD
import FreeCADGui
import os
import re
from draw_danevit_hartenberg import drawDanevitHartenberg
from main_utils import get_robot

class RobotObject:
    def __init__(self, obj):
        # Link the custom object to this proxy
        obj.Proxy = self
        
        obj.addProperty("App::PropertyLinkList", "Constraints", "Robot", "List of links").Constraints = []
        obj.addProperty("App::PropertyLinkList", "Bodies", "Robot", "List of bodies").Bodies = []
        obj.addProperty("App::PropertyIntegerList", "Edges", "Robot", "List of edges").Edges = []
        obj.addProperty("App::PropertyLinkList", "CoordinateSystems", "Robot", "List of coordinate systems").CoordinateSystems = []
        obj.addProperty("App::PropertyLinkList", "BodyCoordinateSystems", "Robot", "List of body coordinate systems").BodyCoordinateSystems = []
        obj.addProperty("App::PropertyIntegerList", "Angles", "Robot", "List of angles").Angles = []

        obj.addProperty("App::PropertyLinkList", "AngleConstraints", "Robot", "List of angle constraints").AngleConstraints = []

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
        if prop == "Angles":
            FreeCAD.Console.PrintMessage(f"Angles updated: {obj.Angles}\n")
            updateAngles()
            drawDanevitHartenberg()



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
    #robot_group = doc.addObject("App::DocumentObjectGroup", "RobotContainer")
    #robot_group.Label = "Robot Container"

    # Create the Robot object
    robot_obj = doc.addObject("App::FeaturePython", "Robot")
    RobotObject(robot_obj)  # Initialize the custom robot object
    RobotViewProvider(robot_obj.ViewObject)  # Attach the view provider

    # Add the Robot object to the group
    doc.Assembly.addObject(robot_obj)

    doc.recompute()

    connectRobotToAssembly()
    drawDanevitHartenberg()
    drawPlanesOnDHCoordinates()

    createAngleConstrains()
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
    
FreeCADGui.addCommand('CreateRobotCommand', CreateRobotCommand())

class Link:
    def __init__(self, joint, body, edge = 0):
        self.Joint = joint
        self.Body = body
        self.Edge = edge

def connectRobotToAssembly():
    doc = FreeCAD.ActiveDocument
    body = lambda ref: ref[1][0].split('.')[0]
    edge = lambda ref: int(re.search(r'\d+$', ref[1][0].split('.')[1]).group()) - 1

    robot = get_robot()
    if robot is None:
        FreeCAD.Console.PrintMessage("No robot object found\n")
        return
        


    #joints = FreeCADGui.Selection.getSelection()
    joints = [j for j in doc.Joints.OutList if hasattr(j, 'JointType') and j.JointType == 'Revolute']

    for obj in doc.Joints.OutList:
        if hasattr(obj, 'ObjectToGround'):
            link_arr = [Link(obj, obj.ObjectToGround)] # Initialize the link array with the grounded joint to astablish the order of the rest

    for _ in joints:
        for joint in joints: # Double loop to add in the right order

            if link_arr and joint.Name not in [link.Joint.Name for link in link_arr]: # If joint is not already in link_arr
                ref1, ref2 = body(joint.Reference1), body(joint.Reference2)
                edge1, edge2 = edge(joint.Reference1), edge(joint.Reference2)

                if link_arr[-1].Body.Name == ref1:
                    link_arr.append(  Link(joint, eval(f"FreeCAD.ActiveDocument.{ref2}"), edge2)  )

                elif link_arr[-1].Body.Name == ref2:
                    link_arr.append(  Link(joint, eval(f"FreeCAD.ActiveDocument.{ref1}"), edge1)  )
                    

    robot.Base = link_arr[0].Body
    link_arr.pop(0) # Remove the grounded joint (Is not a motor)

    robot.Constraints = [link.Joint for link in link_arr]
    robot.Bodies = [link.Body for link in link_arr]
    robot.Edges = [link.Edge for link in link_arr]
    robot.Angles = [0 for _ in link_arr]


def drawPlanesOnDHCoordinates():
    doc = FreeCAD.ActiveDocument
    robot = get_robot()
    for i, body, edge in zip(range(len(robot.Bodies)), robot.Bodies, robot.Edges):
        print(f"Body: {body.Name}, Edge: {edge}")
        lcs = doc.addObject('PartDesign::CoordinateSystem', f'Angle_reference_LCS_{i+1}')
        original_body = body.LinkedObject
        original_body.addObject(lcs)

        circle = doc.getObject(original_body.Name).Shape.Edges[edge].Curve # Finds the circle of the constraint
        lcs.Placement.Base = circle.Center # Sets the base of the coordinate system to the center of the circle

        z_axis = circle.Axis
        temp = FreeCAD.Vector(1,0,0) if z_axis.cross(FreeCAD.Vector(1,0,0)).Length > 1e-6 else FreeCAD.Vector(0,1,0)
        x_axis = z_axis.cross(temp).normalize()
        y_axis = z_axis.cross(x_axis)

        lcs.Placement.Rotation = FreeCAD.Rotation(x_axis, y_axis, z_axis)

        lcs.ViewObject.Visibility = False

        datum_plane = original_body.newObject('PartDesign::Plane', f'plane_on_body_{i+1}')
        datum_plane.AttachmentOffset = FreeCAD.Placement(
            FreeCAD.Vector(0.0, 0.0, 0.0),  # Position the plane
            FreeCAD.Rotation(FreeCAD.Vector(0, 1, 0), 0.0)  # No rotation
        )
        datum_plane.MapReversed = False
        datum_plane.AttachmentSupport = [(lcs, '')]  # Attach to the LCS
        datum_plane.MapPathParameter = 0.0
        datum_plane.MapMode = 'ObjectXZ'  # Align the plane to the X-Z plane of the LCS
        datum_plane.recompute()
        datum_plane.ViewObject.Visibility = False



def createAngleConstrains():
    import FreeCAD as App
    import sys, os

    # Add "Assembly" folder to path dynamically
    sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'Assembly'))

    # Import required classes
    from JointObject import Joint, ViewProviderJoint
    

    doc = App.ActiveDocument

    robot = get_robot()
    angle_constraints = []
    for i, body in enumerate(robot.Bodies):

        print(f" -- Creating joint for body {body.Name} --") # debug

        joint = doc.addObject("App::FeaturePython", "Joint")
        doc.Joints.addObject(joint)

        Joint(joint, 8)  # For example, index 8 for "Angle" joint type
        ViewProviderJoint(joint.ViewObject)
        joint.Activated = True
        joint.Distance = 0.0
        joint.JointType = "Angle"
        joint.Label = f"Robot_joint_{i+1}"
        joint.Reference1 = [App.ActiveDocument.getObject("Assembly"), [f'{body.Name}.plane_on_body_{i+1}.Plane', f'{body.Name}.plane_on_body_{i+1}.']]
        joint.Reference2 = [App.ActiveDocument.getObject("Assembly"), [f'{robot.Base.Name}.plane_on_DH_coordinates_{i+1}.Plane', f'{robot.Base.Name}.plane_on_DH_coordinates_{i+1}.']]
        joint.Visibility = False

        angle_constraints.append(joint)

        print(f'{body.Name}.plane_on_body_{i+1}.Plane')
        print(f'{robot.Base.Name}.plane_on_DH_coordinates_{i+1}.Plane')

    robot.AngleConstraints = angle_constraints
    drawDanevitHartenberg()

    reSolve()

def reSolve():
    import UtilsAssembly
    import FreeCAD as App
    import sys, os

    # Add "Assembly" folder to path dynamically
    sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'Assembly'))

    assembly = UtilsAssembly.activeAssembly()
    if not assembly:
        return

    App.setActiveTransaction("Solve assembly")
    assembly.solve()
    App.closeActiveTransaction()

    FreeCAD.ActiveDocument.recompute()




def updateAngles():
    robot = get_robot()
    for angle, angle_constraint in zip(robot.Angles, robot.AngleConstraints):
        angle_constraint.Distance = angle
    print(f"Angles set to: {robot.Angles}")
    drawDanevitHartenberg()
    reSolve()