import FreeCAD
import FreeCADGui
import os
import re
from main_utils import get_robot


class testCommand:
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
        changeRotationDirection()

    def IsActive(self):
        """Determines if the command is active."""
        # You can add conditions here if needed
        return True if get_robot() != None else False

FreeCADGui.addCommand('testCommand', testCommand())


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

    robot.CoordinateSystems = lcs_arr
    robot.BodyJointCoordinateSystems = BodyJointCoordinateSystems



def updateAngles():
    robot = get_robot()
    lcs_dh_list = robot.CoordinateSystems
    bodies_dh_list = robot.PrevBodies
    lcs_ref_list = robot.BodyJointCoordinateSystems
    bodies_ref_list = robot.Bodies


    for lcs_dh, body_dh, lcs_ref, body_ref, angle in zip(lcs_dh_list, bodies_dh_list, lcs_ref_list, bodies_ref_list, robot.Angles):
        o_A_ol = body_dh.Placement.Matrix
        ol_A_dh = lcs_dh.Placement.Matrix * FreeCAD.Rotation(FreeCAD.Vector(0,0,1), angle).toMatrix()
        b_A_dh = lcs_ref.Placement.Matrix
        o_A_b =  o_A_ol * ol_A_dh * b_A_dh.inverse()
        
        body_ref.Placement.Matrix =  o_A_b 
        
        




class flipDHCommand:
    """Command to draw the robot using Denavit-Hartenberg parameters."""

    def __init__(self):
        pass

    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'rotateBody.svg'),
            'MenuText': 'Rotate Joint',
            'ToolTip': 'Rotate the zero-state of a joint 90 degrees'
        }

    def Activated(self):
        flipDH()

    def IsActive(self):
        if get_robot() == None:
            return False
        
        sel = FreeCADGui.Selection.getSelection()
        if not sel:
            return False
        
        for s in sel:
            if s not in get_robot().Bodies:
                return False

        return True

FreeCADGui.addCommand('flipDHCommand', flipDHCommand())

def flipDH():
    robot = get_robot()
    selections = FreeCADGui.Selection.getSelection()

    for sel in selections:
        body = sel
        idx = robot.Bodies.index(body)
        print(f"Flipping {body.Name}, idx: {idx}")
        robot.BodyJointCoordinateSystems[idx].Placement.Rotation = robot.BodyJointCoordinateSystems[idx].Placement.Rotation * FreeCAD.Rotation(FreeCAD.Vector(0,0,1), 90)
        if idx+1 < len(robot.CoordinateSystems):
            robot.CoordinateSystems[idx+1].Placement.Rotation = robot.CoordinateSystems[idx+1].Placement.Rotation * FreeCAD.Rotation(FreeCAD.Vector(0,0,1), 90)
        updateAngles()





class changeRotationDirectionCommand:
    """Command to draw the robot using Denavit-Hartenberg parameters."""

    def __init__(self):
        pass

    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'changeDirection.svg'),
            'MenuText': 'Change Rotation Direction',
            'ToolTip': 'Change the positive rotation direction of the selected joint'
        }

    def Activated(self):
        changeRotationDirection()

    def IsActive(self):
        if get_robot() == None:
            return False
        
        sel = FreeCADGui.Selection.getSelection()

        if not sel:
            return False
    
        for s in sel:
            if s not in get_robot().Bodies:
                return False

        return True

FreeCADGui.addCommand('changeRotationDirectionCommand', changeRotationDirectionCommand())

def changeRotationDirection():
    robot = get_robot()
    selections = FreeCADGui.Selection.getSelection()

    for sel in selections:
        body = sel
        idx = robot.Bodies.index(body)
        print(f"Flipping DH Coordinate system {body.Name}, idx: {idx}")
        cs = robot.CoordinateSystems[idx]
        cs.Placement.Rotation = cs.Placement.Rotation * FreeCAD.Rotation(FreeCAD.Vector(1, 0, 0), 180) # Swap direction of the z-axis

        cs_ref = robot.BodyJointCoordinateSystems[idx]
        cs_ref.Placement.Rotation = cs_ref.Placement.Rotation * FreeCAD.Rotation(FreeCAD.Vector(1,0,0), 180)

        updateAngles()



        
    