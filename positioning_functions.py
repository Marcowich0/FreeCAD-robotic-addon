import FreeCAD
import FreeCADGui
import os
import re
from main_utils import get_robot, correctLinkPosition


class testingCommand:
    """Command to draw the robot using Denavit-Hartenberg parameters."""

    def __init__(self):
        pass

    def GetResources(self):
        """Returns the resources associated with the command."""
        return {
            'MenuText': 'Draw Denavit-Hartenberg',
            'ToolTip': 'Draw the robot using Denavit-Hartenberg parameters',
            'Pixmap': 'path/to/icon.svg'  # Provide the path to your icon
        }

    def Activated(self):
        """Called when the command is activated (e.g., button pressed)."""
        #drawDanevitHartenberg()
        updateDanevitHartenberg()

    def IsActive(self):
        """Determines if the command is active."""
        # You can add conditions here if needed
        return True if get_robot() != None else False

FreeCADGui.addCommand('testingCommand', testingCommand())


def createLocalCoordinateOnBodies():
    doc = FreeCAD.ActiveDocument
    robot = get_robot()
    BodyJointCoordinateSystems = []
    for i, body, edge in zip(range(len(robot.Bodies)), robot.Bodies, robot.Edges):
        print(f"Body: {body.Name}, Edge: {edge}")
        lcs = doc.addObject('PartDesign::CoordinateSystem', f'Angle_reference_LCS_{i}')
        original_body = body.LinkedObject
        original_body.addObject(lcs)

        circle = doc.getObject(original_body.Name).Shape.Edges[edge].Curve # Finds the circle of the constraint
        lcs.Placement.Base = circle.Center # Sets the base of the coordinate system to the center of the circle

        o_A_dh = doc.getObject(f"LCS_link_{i}").Placement.Rotation
        o_A_b  = body.Placement.Rotation
        b_A_dh = o_A_b.inverted() * o_A_dh
        
        lcs.Placement.Rotation = b_A_dh
        BodyJointCoordinateSystems.append(lcs)

    robot.BodyJointCoordinateSystems = BodyJointCoordinateSystems
        
        

def createDanevitHartenberg():
    doc = FreeCAD.ActiveDocument
    obj = get_robot()
    """Draw the robot using the Denavit-Hartenberg parameters."""
    robot = get_robot()
    prev_bodies = robot.PrevBodies
    prev_edges = robot.PrevEdges

    lcs_dh = doc.addObject( 'PartDesign::CoordinateSystem', f'LCS_link_0' )
    prev_bodies[0].LinkedObject.addObject(lcs_dh)
    lcs_dh.Placement.Base = prev_bodies[0].LinkedObject.Shape.Edges[prev_edges[0]].Curve.Center

    lcs_arr = [lcs_dh]

    BodyJointCoordinateSystems = []

    for i, body in zip(range(1,len(obj.Constraints)+1), obj.Bodies):

        print(f"Creating LCS for joint {i}") # debug
        # Creating ref to last dh
        lcs_ref = doc.addObject('PartDesign::CoordinateSystem', f'Angle_reference_LCS_{i-1}')
        body.LinkedObject.addObject(lcs_ref)
        
        o_A_ol = prev_bodies[i-1].Placement.Matrix

        print(f" Angle_reference_LCS_{i-1},   o_A_ol: {o_A_ol}") # debug
        ol_A_dh = lcs_arr[-1].Placement.Matrix
        o_A_b  = body.Placement.Matrix

        b_A_dh = o_A_b.inverse() * o_A_ol * ol_A_dh
        
        lcs_ref.Placement.Matrix = b_A_dh
        BodyJointCoordinateSystems.append(lcs_ref)

        
        if i < len(obj.Constraints):
            print(f" -- Creating DH for joint {i}") # debug
            lcs_dh = doc.addObject( 'PartDesign::CoordinateSystem', f'LCS_link_{i}' ) # Adds coordinate system to the document
            body.LinkedObject.addObject(lcs_dh)

            lcs_dh.Placement.Base = body.LinkedObject.Shape.Edges[prev_edges[i]].Curve.Center # Sets the base of the coordinate system to the center of the circle
            
            last_x = lcs_ref.Placement.Rotation.multVec(FreeCAD.Vector(1,0,0)) 
            last_z = lcs_ref.Placement.Rotation.multVec(FreeCAD.Vector(0,0,1)) 
            
            circle = body.LinkedObject.Shape.Edges[prev_edges[i]].Curve # Finds the circle of the constraint
            parallel = last_z.cross(circle.Axis).Length < 1e-6 # Checks if the circle axis is parallel to the last z-axis
            print(f"parallel: {parallel}") # debug
            if parallel:
                z_axis = last_z
                x_axis = last_x

            else:
                z_axis = circle.Axis # Sets the axis of the coordinate system to the axis of the circle
                x_axis = last_z.cross(z_axis)

            y_axis = z_axis.cross(x_axis)

            lcs_dh.Placement.Rotation =  FreeCAD.Rotation(x_axis, y_axis, z_axis)

            lcs_arr.append(lcs_dh)

    obj.CoordinateSystems = lcs_arr



def updateDanevitHartenberg():
    doc = FreeCAD.ActiveDocument
    robot = get_robot()

    test_angles = [10, 20, 30, 40]
    for i, angle in enumerate(robot.Angles):

        
        o_A_dh = robot.CoordinateSystems[i+1].Placement.Matrix
        b_A_c = robot.BodyJointCoordinateSystems[i].Placement.Matrix

        body = robot.Bodies[i]
        edge = robot.Edges[i]
        body.Placement.Matrix = o_A_dh * b_A_c.inverse()

        print(f" -- Updating LCS for joint {i}")

        current_dh = robot.CoordinateSystems[i+2]
        current_dh.Placement.Base = body.Shape.Edges[edge].Curve.Center
        print(f"Current oriontation: {current_dh.Placement.Matrix}") # debug
        current_dh.Placement.Rotation = current_dh.Placement.Rotation.multiply(FreeCAD.Rotation(FreeCAD.Vector(0,0,1), test_angles[i]))
        print(f"New oriontation: {current_dh.Placement.Matrix}") # debug


        

        
    