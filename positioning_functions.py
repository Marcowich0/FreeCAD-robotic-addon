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
        solve_ik_nsolve_system(720, 120, 0)

    def IsActive(self):
        """Determines if the command is active."""
        # You can add conditions here if needed
        return True if get_robot() != None else False

FreeCADGui.addCommand('testCommand', testCommand())


def createDanevitHartenberg():
    doc, robot = FreeCAD.ActiveDocument, get_robot()
    prev_bodies, prev_edges = robot.PrevBodies, robot.PrevEdges
    lcs_arr = [doc.addObject('PartDesign::CoordinateSystem', 'LCS_link_0')]
    prev_bodies[0].LinkedObject.addObject(lcs_arr[0])
    lcs_arr[0].Placement.Base = prev_bodies[0].LinkedObject.Shape.Edges[prev_edges[0]].Curve.Center
    BodyJointCoordinateSystems = []

    for i, body in zip(range(1,len(robot.Constraints)+1), robot.Bodies):
        print(f"Creating LCS for joint {i}") 
        lcs_ref = doc.addObject('PartDesign::CoordinateSystem', f'Angle_reference_LCS_{i-1}')
        body.LinkedObject.addObject(lcs_ref)
        lcs_ref.ViewObject.Visibility = False
        
        o_A_ol = prev_bodies[i-1].Placement.Matrix      # Local origo of previous body to glocal origo
        ol_A_dh = lcs_arr[-1].Placement.Matrix          # Danevit-Hartenberg to local origo 
        o_A_b  = body.Placement.Matrix                  # Local origo of current body to global origo
        b_A_dh = o_A_b.inverse() * o_A_ol * ol_A_dh     # Danevit-Hartenberg in current body coordinate system
        
        lcs_ref.Placement.Matrix = b_A_dh
        BodyJointCoordinateSystems.append(lcs_ref)

        if i < len(robot.Constraints):
            print(f" -- Creating DH for joint {i}") # debug
            lcs_dh = doc.addObject( 'PartDesign::CoordinateSystem', f'LCS_link_{i}' ) 
            body.LinkedObject.addObject(lcs_dh)

            lcs_dh.Placement.Base = body.LinkedObject.Shape.Edges[prev_edges[i]].Curve.Center # Sets the base of the coordinate system to the center of the circle
            
            last_x = lcs_ref.Placement.Rotation.multVec(FreeCAD.Vector(1,0,0)) 
            last_z = lcs_ref.Placement.Rotation.multVec(FreeCAD.Vector(0,0,1)) 
            circle = body.LinkedObject.Shape.Edges[prev_edges[i]].Curve 
            
            if last_z.cross(circle.Axis).Length < 1e-6: # If the circle axis is parallel to the last z-axis
                z_axis = last_z
                x_axis = last_x
            else:
                z_axis = circle.Axis # Sets the axis of the coordinate system to the axis of the circle
                x_axis = last_z.cross(z_axis)

            y_axis = z_axis.cross(x_axis)
            lcs_dh.Placement.Rotation = FreeCAD.Rotation(x_axis, y_axis, z_axis)

            lcs_arr.append(lcs_dh)

    robot.CoordinateSystems = lcs_arr
    robot.BodyJointCoordinateSystems = BodyJointCoordinateSystems




def updateAngles():
    robot = get_robot()
    lcs_dh_list = robot.CoordinateSystems
    bodies_dh_list = robot.PrevBodies
    lcs_ref_list = robot.BodyJointCoordinateSystems
    bodies_ref_list = robot.Bodies

    robot.Angles = [(angle + 180) % 360 - 180 for angle in robot.Angles]


    for lcs_dh, body_dh, lcs_ref, body_ref, angle in zip(lcs_dh_list, bodies_dh_list, lcs_ref_list, bodies_ref_list, robot.Angles):
        o_A_ol = body_dh.Placement.Matrix
        ol_A_dh = lcs_dh.Placement.Matrix * FreeCAD.Rotation(FreeCAD.Vector(0,0,1), angle).toMatrix()
        b_A_dh = lcs_ref.Placement.Matrix
        o_A_b =  o_A_ol * ol_A_dh * b_A_dh.inverse()
        
        body_ref.Placement.Matrix =  o_A_b 
        

######################################################################################################

class rotateBodyZeroCommand:
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
        rotateBodyZero()

    def IsActive(self):        
        sel = FreeCADGui.Selection.getSelection()
        if not sel or get_robot() == None:
            return False
        
        for s in sel:
            if s not in get_robot().Bodies:
                return False
        return True

FreeCADGui.addCommand('rotateBodyZeroCommand', rotateBodyZeroCommand())


from main_utils import mat_to_numpy, numpy_to_mat, np_rotation, numpy_to_rotation
import numpy as np

def rotateBodyZero():
    robot = get_robot()
    body = FreeCADGui.Selection.getSelection()[0]

    idx = robot.Bodies.index(body)
    print(f"Flipping {body.Name}, idx: {idx}")

    o1_A_ref = mat_to_numpy(robot.BodyJointCoordinateSystems[idx].Placement.Matrix)
    o1_A_dh = mat_to_numpy(robot.CoordinateSystems[idx+1].Placement.Matrix)
    
    ref_A_dh = np.linalg.inv(o1_A_ref) @ o1_A_dh

    o1_A_ref = o1_A_ref @ np_rotation(-np.pi/2, 'z')
    ref_A_dh = np_rotation(np.pi/2, 'z') @ ref_A_dh
    o1_A_dh = o1_A_ref @ ref_A_dh

    robot.BodyJointCoordinateSystems[idx].Placement.Rotation = FreeCAD.Placement(numpy_to_mat(o1_A_ref)).Rotation
    robot.CoordinateSystems[idx+1].Placement.Rotation = FreeCAD.Placement(numpy_to_mat(o1_A_dh)).Rotation

    updateAngles()



##################################################################################################################

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
        sel = FreeCADGui.Selection.getSelection()

        if not sel or get_robot() == None:
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
        robot.CoordinateSystems[idx].Placement.Rotation *= FreeCAD.Rotation(FreeCAD.Vector(1, 0, 0), 180) # Swap direction of the z-axis
        robot.BodyJointCoordinateSystems[idx].Placement.Rotation *= FreeCAD.Rotation(FreeCAD.Vector(1,0,0), 180)

        updateAngles()



        
##################################################################################################

class defineEndEffectorCommand:
    """Command to draw the robot using Denavit-Hartenberg parameters."""

    def __init__(self):
        pass

    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'endEffector.svg'),
            'MenuText': 'Define End Effector',
            'ToolTip': 'Defines the position of the end effector based on a point on the last link'
        }

    def Activated(self):
        defineEndEffector()

    def IsActive(self):        
        sel = FreeCADGui.Selection.getSelection()

        sel = FreeCADGui.Selection.getSelectionEx()
        if sel:
            sobj = sel[0]
            sub_objects = sobj.SubObjects
            
            if sub_objects:
                first_subobj = sub_objects[0]
                
                # For a vertex, you can get its 3D coordinates:
                if first_subobj.ShapeType == "Vertex":
                    return True

        return False

FreeCADGui.addCommand('defineEndEffectorCommand', defineEndEffectorCommand())



def defineEndEffector():
    robot = get_robot()
    sel = FreeCADGui.Selection.getSelectionEx()

    if sel:
        sobj = sel[0]
        obj = sobj.Object
        sub_names = sobj.SubElementNames
        sub_objects = sobj.SubObjects
        
        if sub_objects:
            first_subobj = sub_objects[0]
            
            # For a vertex, you can get its 3D coordinates:
            if first_subobj.ShapeType == "Vertex":
                print("Vertex coordinates:", first_subobj.Point)
                robot.EndEffector = first_subobj.Point





##################################################################################################

import sympy as sp

def defineSympyDHTranformationMatrices():
    robot = get_robot()
    DH_perameters = findDHPerameters()

    T_arr = [sp.Matrix([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])]
    for theta, d, a, alpha in DH_perameters:
        DH_mat = sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
                            [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
                            [0, sp.sin(alpha), sp.cos(alpha), d],
                            [0, 0, 0, 1]])
        T_arr.append(T_arr[-1]*DH_mat)

    end = sp.Matrix([[0,0,0,robot.EndEffector.x],
                    [0,0,0,robot.EndEffector.y],
                    [0,0,0,robot.EndEffector.z],
                    [0,0,0,1]])
    T_arr.append(T_arr[-1]*end)
    print(T_arr[-1]*sp.Matrix([0,0,0,1]))
    T_arr.pop(0)
    T_arr = [sp.simplify(T) for T in T_arr]
    return T_arr



from sympy import nsolve

def solve_ik_nsolve_system(x_des, y_des, z_des):
    robot = get_robot()
    T_arr = defineSympyDHTranformationMatrices()
    theta_syms = [sp.symbols(f'theta_{i}') for i in range(len(robot.Constraints))]
    p_end = (T_arr[-1]*sp.Matrix([0, 0, 0, 1]))

    eqs = [
        p_end[0] - x_des,
        p_end[1] - y_des,
        p_end[2] - z_des
    ]
    # 'init_guess' must be a tuple of initial guesses matching 'theta_syms'
    sol = nsolve(eqs, theta_syms, robot.Angles)
    # nsolve_system returns (sol_theta0, sol_theta1, ...)
    print({theta_syms[i]: float(sol[i]) for i in range(len(theta_syms))})
    return {theta_syms[i]: float(sol[i]) for i in range(len(theta_syms))}




##################################################################################################



class FindDHParametersCommand:
    """A FreeCAD command to calculate Danevit Hartenberg parameters based on robot object."""

    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'danevitHartenberg.svg'),
            'MenuText': 'Calculate DH Parameters',
            'ToolTip': 'Finds the Danevit Hartenberg parameters for the robot'
        }

    def Activated(self):
        """Called when the command is activated (button clicked)."""
        print(findDHPerameters())

    def IsActive(self):
        """Determine if the command should be active."""
        return True if get_robot() != None else False
    
FreeCADGui.addCommand('FindDHParametersCommand', FindDHParametersCommand())


def findDHPerameters():
    import sympy as sp
    robot = get_robot()
    old_angles = robot.Angles
    robot.Angles = [0 for _ in robot.Angles]

    DH_transformations = []
    DH_parameters = []

    for body, lcs_ref in zip(robot.Bodies, robot.BodyJointCoordinateSystems):
        o_A_ol = body.Placement.Matrix 
        ol_A_dh = lcs_ref.Placement.Matrix
        o_A_dh = o_A_ol * ol_A_dh
        DH_transformations.append(np.array(o_A_dh.A).reshape(4, 4))
    

    theta = [sp.symbols(f'theta_{i}') for i in range(len(robot.Constraints))]
    for i, DH in enumerate(DH_transformations):
        DH_sympy = sp.Matrix(DH)
        
        d = round(DH_sympy[2, 3], 3)
        a = round(DH_sympy[0, 3], 3)

        tmp1 = sp.acos(DH_sympy[2, 2])
        tmp2 = sp.asin(DH_sympy[2, 1])
        alpha_val = tmp1 if tmp1 == tmp2 else -tmp1
        alpha = round(alpha_val/np.pi*180, 3)
        
        DH_parameters.append([theta[i], d, a, alpha])
    robot.Angles = old_angles
    return DH_parameters