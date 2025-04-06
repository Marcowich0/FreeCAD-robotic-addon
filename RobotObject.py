import FreeCAD
import FreeCADGui
import os
import re
from forward_kinematics import InitializeCoordinateSystems, positionBodies, positionDHCoordinateSystems, getJacobian, getJacobianCenter
from dynamics import updateMomentOfInertia, defineCenterOffMass, computeJointTorques
from main_utils import get_robot, displayMatrix
import numpy as np

class RobotObject:
    def __init__(self, obj):
        # Link the custom object to this proxy
        obj.Proxy = self
        
        obj.addProperty("App::PropertyLinkList", "Constraints", "Robot", "List of links").Constraints = []
        
        obj.addProperty("App::PropertyIntegerList", "Edges", "Robot", "List of edges").Edges = []
        obj.addProperty("App::PropertyIntegerList", "PrevEdges", "Robot", "List of previous edges").PrevEdges = []
        
        obj.addProperty("App::PropertyFloatList", "Angles", "Robot", "List of angles").Angles = []
        obj.addProperty("App::PropertyFloatList", "AngleOffsets", "Robot", "List of angles offsets").AngleOffsets = []

        obj.addProperty("App::PropertyVector", "EndEffector", "Robot", "End effector position").EndEffector = FreeCAD.Vector(0, 0, 0)
        obj.addProperty("App::PropertyVector", "EndEffectorOrientation", "Robot", "End effector orientation").EndEffectorOrientation = FreeCAD.Vector(0, 0, 0)

        obj.addProperty("App::PropertyLinkList", "Links", "Robot", "List of links").Links = []
        obj.addProperty("App::PropertyPythonObject", "DHPerameters", "Robot", "DH parameters").DHPerameters = []
        obj.addProperty("App::PropertyLinkList", "DHLocalCoordinateSystems", "Robot", "DH Local Coordinate Systems").DHLocalCoordinateSystems = []
        obj.addProperty("App::PropertyLinkList", "DHCoordinateSystems", "Robot", "DH Coordinate Systems").DHCoordinateSystems = []
        obj.addProperty("App::PropertyPythonObject", "InertiaMatrices", "Robot", "Inertia matrices").InertiaMatrices = []
        obj.addProperty("App::PropertyPythonObject", "CenterOfMass", "Robot", "Center of mass").CenterOfMass = []
        obj.addProperty("App::PropertyFloatList", "Masses", "Robot", "Masses of the links").Masses = []


        obj.addProperty("App::PropertyString", "Type", "Base", "Type of the object").Type = "Robot"

        obj.setEditorMode("Type", 1)  # Make the property read-only
        obj.setEditorMode("PrevEdges", 2)  # Make the property invisible
        

    def execute(self, obj):
        """Define how the object behaves when updated."""
        FreeCAD.Console.PrintMessage("Executing RobotObject\n")
    
    def onChanged(self, obj, prop):
        """Handle property changes."""
        if prop == "Angles":
            if obj.DHLocalCoordinateSystems:
                positionDHCoordinateSystems()
                positionBodies()
            
    def __getstate__(self):
        # 1) Copy the current __dict__
        state = self.__dict__.copy()
        # 2) Remove any references to FreeCAD objects that aren’t JSON-serializable
        #    The easiest approach is typically to remove 'obj' or 'Object', if stored:
        if "Object" in state:
            del state["Object"]


        return state

    def __setstate__(self, state):
        # Reassign the pruned or saved attributes back
        self.__dict__.update(state)


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
    
    def __getstate__(self):
        state = self.__dict__.copy()
        # If you store references to the raw FreeCAD object or anything
        # that has a `ViewProviderDocumentObject`, remove it:
        if "Object" in state:
            del state["Object"]
        return state

    def __setstate__(self, state):
        self.__dict__.update(state)




# ----------------- Adding the GUI Button to create robot -----------------

class CreateRobotCommand:
    """A FreeCAD command to create a Robot object."""

    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'robotArm2.svg'),
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


import threading

def initialize_robot():
    """Directly initialize the robot object in the active document."""
    doc = FreeCAD.ActiveDocument
    if doc is None:
        doc = FreeCAD.newDocument()

    # Create the Robot object
    robot_obj = doc.addObject("App::FeaturePython", "Robot")
    RobotObject(robot_obj)  # Initialize the custom robot object
    RobotViewProvider(robot_obj.ViewObject)  # Attach the view provider

    # Add the Robot object to the group
    doc.Assembly.addObject(robot_obj)
    doc.recompute()

    connectRobotToAssembly()
    InitializeCoordinateSystems()

    t = threading.Thread(target=thread_sympy)
    t.start()

    return robot_obj


def thread_sympy():
    defineCenterOffMass()
    updateMomentOfInertia()

# ----------------- Adding the GUI Button to remove robot -----------------

class RemoveRobotCommand:
    """A FreeCAD command to create a Robot object."""

    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'trash.svg'),
            'MenuText': 'Remove Robot',
            'ToolTip': 'Remove all robot elements'
        }

    def Activated(self):
        """Called when the command is activated (button clicked)."""
        remove_robot()

    def IsActive(self):
        """Determine if the command should be active."""
        if get_robot() is None:
            return False
        return True
    
FreeCADGui.addCommand('RemoveRobotCommand', RemoveRobotCommand())

def remove_robot():
    """Remove the robot object from the active document."""
    doc = FreeCAD.ActiveDocument
    robot = get_robot()
    for lcs in robot.DHLocalCoordinateSystems:
        doc.removeObject(lcs.Name)
    for cs in robot.DHCoordinateSystems:
        doc.removeObject(cs.Name)
    doc.removeObject("DH_coordinate_systems")
    doc.removeObject(robot.Name)
    doc.recompute()







# ----------------- Connecting the Robot to the Assembly -----------------
class Link:
    def __init__(self, joint, body, edge = 0):
        self.Joint = joint
        self.Body = body
        self.Edge = edge

def connectRobotToAssembly():
    doc = FreeCAD.ActiveDocument
    body = lambda ref: ref[1][0].split('.')[0]
    edge = lambda ref: int(re.search(r'Edge(\d+)', ref[1][0]).group(1)) - 1

    robot = get_robot()
    if robot is None:
        FreeCAD.Console.PrintMessage("No robot object found\n")
        return
        
    joints = [j for j in doc.Joints.OutList if hasattr(j, 'JointType') and j.JointType == 'Revolute'] # Get all the revolute joints

    for obj in doc.Joints.OutList:
        if hasattr(obj, 'ObjectToGround'):
            # Correct the placement of the first body to the identity matrix to have consistent baseframe with assembly
            first_body = obj.ObjectToGround
            identity_matrix = FreeCAD.Matrix(
                FreeCAD.Vector(1, 0, 0),
                FreeCAD.Vector(0, 1, 0),
                FreeCAD.Vector(0, 0, 1),
                FreeCAD.Vector(0, 0, 0)  # Translation vector
            )
            obj.ObjectToGround = None
            first_body.Placement.Matrix = identity_matrix
            obj.ObjectToGround = first_body

            link_arr = [Link(obj, first_body)] # Initialize the link array with the grounded joint to astablish the order of the rest
            link_prev_arr = []
            print(f"Corrected placement of {first_body.Name} to {first_body.Placement.Matrix}\n")
            
    for _ in joints:
        for joint in joints: # Double loop to add in the right order

            if link_arr and joint.Name not in [link.Joint.Name for link in link_arr]: # If joint is not already in link_arr
                ref1, ref2 = body(joint.Reference1), body(joint.Reference2)
                edge1, edge2 = edge(joint.Reference1), edge(joint.Reference2)

                if link_arr[-1].Body.Name == ref1:
                    link_arr.append(  Link(joint, eval(f"FreeCAD.ActiveDocument.{ref2}"), edge2)  )
                    link_prev_arr.append(  Link(joint, eval(f"FreeCAD.ActiveDocument.{ref1}"), edge1)  )

                elif link_arr[-1].Body.Name == ref2:
                    link_arr.append(  Link(joint, eval(f"FreeCAD.ActiveDocument.{ref1}"), edge1)  )
                    link_prev_arr.append(  Link(joint, eval(f"FreeCAD.ActiveDocument.{ref2}"), edge2)  )
                    

    robot.Links = [link.Body for link in link_arr]
    link_arr.pop(0) # Remove the grounded joint (Is not a motor)

    robot.Constraints = [link.Joint for link in link_arr]
    robot.Edges = [link.Edge for link in link_arr]
    robot.Angles = [0 for _ in link_arr]
    robot.AngleOffsets = [0 for _ in link_arr]
    robot.PrevEdges = [link.Edge for link in link_prev_arr]


    for constraint in robot.Constraints: # Deactivate the constraints so the assembly does not solve the joints
        constraint.Activated = False


