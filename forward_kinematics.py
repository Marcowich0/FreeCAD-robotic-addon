import FreeCAD
import FreeCADGui
import os
import re
from main_utils import get_robot, updateGlobalEndEffector, vec_to_numpy
from main_utils import mat_to_numpy, numpy_to_mat, np_rotation, numpy_to_rotation
import numpy as np

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
  

    def IsActive(self):
        """Determines if the command is active."""
        # You can add conditions here if needed
        return True if get_robot() != None else False

FreeCADGui.addCommand('testCommand', testCommand())

def CreateLocalDHCoordinateSystems():
    doc, robot = FreeCAD.ActiveDocument, get_robot()
    edges = robot.PrevEdges
    last = np.array([[1,0,0,0],
                    [0,1,0,0],
                    [0,0,1,0],
                    [0,0,0,1]])
    local_dh_coordinate_systems = []
    for i, link in enumerate(robot.Links):
        o_A_ol = mat_to_numpy(link.Placement.Matrix) 
        ol_A_last = np.linalg.inv(o_A_ol) @ last
        lcs = doc.addObject('PartDesign::CoordinateSystem', f'LCS_DH_{i}')
        link.LinkedObject.addObject(lcs)

        if i<len(robot.Links)-1:
            origo = vec_to_numpy(link.LinkedObject.Shape.Edges[edges[i]].Curve.Center)

            last_x = ol_A_last[0:3, 0]
            last_z = ol_A_last[0:3, 2]
            circle = link.LinkedObject.Shape.Edges[edges[i]].Curve 
            if np.linalg.norm( np.cross(last_z, vec_to_numpy(circle.Axis)) ) < 1e-6: # If the circle axis is parallel to the last z-axis
                z_axis = last_z
                x_axis = last_x
            else:
                z_axis = vec_to_numpy(circle.Axis) # Sets the axis of the coordinate system to the axis of the circle
                x_axis = np.cross(last_z, z_axis)

            y_axis = np.cross(z_axis, x_axis)

        else: # If it is the last link, set the end effector as the origo
            origo = vec_to_numpy(robot.EndEffector)
            x_axis = ol_A_last[0:3, 0]
            y_axis = ol_A_last[0:3, 1]
            z_axis = ol_A_last[0:3, 2]


        old_z = last_z
        new_z = z_axis
        p1 = sp.Matrix(ol_A_last[0:3, 3])
        d1 = sp.Matrix(ol_A_last[0:3, 2]).normalized()
        p2 = sp.Matrix(origo)
        d2 = sp.Matrix(z_axis).normalized()
        
        
        if np.linalg.norm(np.cross(old_z, new_z)) > 1e-6:
            print("Non-parallel z-axes")
            t, s = sp.symbols('t s', real=True)
            eq1 = sp.Eq(d1.dot(p1 - p2 + t*d1 - s*d2), 0)
            eq2 = sp.Eq(d2.dot(p1 - p2 + t*d1 - s*d2), 0)
            sol = sp.solve([eq1, eq2], (t, s), dict=True)
            sol = sol[0]
            print(sol)
            translation = float(sol[s])
        else:
            print("Parallel z-axes")
            translation = float(p1[2]-p2[2])
        print(f"link: {link.Name}")
        #print(f"p1: {p1}, d1: {d1}, p2: {p2}, d2: {d2}")

        translation_mat = np.array([[1,0,0,0],
                                    [0,1,0,0],
                                    [0,0,1, translation],
                                    [0,0,0,1]])        
        
        ol_A_dh = np.array([[x_axis[0], y_axis[0], z_axis[0], origo[0]],
                            [x_axis[1], y_axis[1], z_axis[1], origo[1]],
                            [x_axis[2], y_axis[2], z_axis[2], origo[2]],
                            [0, 0, 0, 1]]) @ translation_mat


        lcs.Placement.Matrix = numpy_to_mat(ol_A_dh)


        # Rotate body to allight the x-axes
        last_x = ol_A_last[0:3, 0]
        current_x = ol_A_dh[0:3, 0]
        cos_angle = np.dot(last_x, current_x) / (np.linalg.norm(last_x) * np.linalg.norm(current_x))
        sin_angle = np.linalg.norm(np.cross(last_x, current_x)) / (np.linalg.norm(last_x) * np.linalg.norm(current_x))
        angle = np.arctan2(sin_angle, cos_angle)
        if np.dot(np.cross(last_x, current_x), last_z) < 0:
            angle = -angle
        angle_between = angle
        print(f"Angle between x-axes: {angle_between}")
        print(f"Axis of rotation: {last[0:3, 2]}")
        print(f"point of rotation: {last[0:3, 3]}")

        for link2 in robot.Links[i:]:
            o_A_o2 = mat_to_numpy(link2.Placement.Matrix)
            o2_A_last = np.linalg.inv(o_A_o2) @ last
            link2.Placement.Matrix = numpy_to_mat( last @ np_rotation(-angle_between, 'z') @ np.linalg.inv(o2_A_last) )

        last = mat_to_numpy(link.Placement.Matrix) @ mat_to_numpy(lcs.Placement.Matrix)
        local_dh_coordinate_systems.append(lcs)

    robot.DHLocalCoordinateSystems = local_dh_coordinate_systems

    findDHPeremeters()
    createDHCoordinateSystems()
    updateDHTransformations()
    positionDHCoordinateSystems()
    updateJacobian()



def positionBodies():
    robot = get_robot()
    robot.Angles = [(angle + 180) % 360 - 180 for angle in robot.Angles]

    for theory_dh, freecad_dh, body in zip(robot.DHCoordinateSystems[1:], robot.DHLocalCoordinateSystems[1:], robot.Bodies):
        o_A_theory = mat_to_numpy(theory_dh.Placement.Matrix)
        ol_A_freecad = mat_to_numpy(freecad_dh.Placement.Matrix)
        body.Placement.Matrix = numpy_to_mat(o_A_theory @ np.linalg.inv(ol_A_freecad))
        



        
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
    old_angles = robot.Angles
    robot.Angles = [0 for _ in robot.Angles]
    sel = FreeCADGui.Selection.getSelectionEx()

    if sel:
        sobj = sel[0]
        sub_objects = sobj.SubObjects
        
        if sub_objects:
            first_subobj = sub_objects[0]
            
            # For a vertex, you can get its 3D coordinates:
            if first_subobj.ShapeType == "Vertex":
                print("Vertex coordinates:", first_subobj.Point)
                robot.EndEffector = first_subobj.Point
                robot.DHLocalCoordinateSystems[-1].Placement.Base = first_subobj.Point
                findDHPeremeters()
                updateDHTransformations()
                positionDHCoordinateSystems()

    robot.Angles = old_angles





##################################################################################################

import sympy as sp

"""
def defineTranformationMatrices():
    robot = get_robot()
    theta = robot.sympyVariables
    old_angles = robot.Angles
    robot.Angles = [0 for _ in robot.Angles]

    T_arr = [sp.Matrix(mat_to_numpy(robot.CoordinateSystems[0].Placement.Matrix))]

    for i, (lcs_ref, lcs_dh) in enumerate(zip(robot.BodyJointCoordinateSystems, robot.CoordinateSystems[1:])):
        print(f"Creating transformation matrix for joint {i}, {lcs_ref.Name} -> {lcs_dh.Name}")
        ol_A_ref = mat_to_numpy(lcs_ref.Placement.Matrix)
        ol_A_dh = mat_to_numpy(lcs_dh.Placement.Matrix)
        ref_A_dh = np.linalg.inv(ol_A_ref) @ ol_A_dh

        rot = sp.Matrix([[sp.cos(theta[i]), -sp.sin(theta[i]), 0, 0],
                         [sp.sin(theta[i]), sp.cos(theta[i]), 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
        
        T_arr.append(T_arr[-1] * rot * sp.Matrix(ref_A_dh)) 

    ol_A_lc = mat_to_numpy(robot.BodyJointCoordinateSystems[-1].Placement.Matrix)
    ol_P = np.array([[0,0,0,robot.EndEffector.x],
                       [0,0,0,robot.EndEffector.y],
                       [0,0,0,robot.EndEffector.z],
                       [0,0,0,1]])
    
    T_arr.append(T_arr[-1] * np.linalg.inv(ol_A_lc) @ ol_P)
    robot.SympyTransformations = T_arr
    robot.NumpyTransformations = [sp.lambdify(theta, T, 'numpy') for T in T_arr]
    robot.Angles = old_angles
    updateGlobalEndEffector()
"""

def updateJacobian():
    robot = get_robot()
    T_arr = robot.DHTransformations
    for T in T_arr:
        print(T)
    Jac = []
    On = T_arr[-1][0:3, 3]
    for i in range(1, len(T_arr)):
        Zi = T_arr[i-1][0:3, 2]
        Oi = T_arr[i-1][0:3, 3]
        Jv = np.cross( Zi , (On - Oi)  )
        Jw = Zi
        Jac.append([*Jv, *Jw])
    robot.Jacobian = np.array(Jac).T
    print(np.shape(robot.Jacobian))



##################################################################################################


def findDHPeremeters():
    robot = get_robot()
    DH_peremeters = []
    #robot.Angles = [0 for _ in robot.Angles]

    last_lcs = robot.DHLocalCoordinateSystems[:len(robot.Links)-1]
    last_body = robot.Links[:len(robot.Links)-1]
    current_lcs = robot.DHLocalCoordinateSystems[1:]
    current_body = robot.Links[1:]

    for i, last_lcs, last_body, current_lcs, current_body in zip(range(1,len(robot.Links)), last_lcs, last_body, current_lcs, current_body):
        o_A_last = mat_to_numpy(last_body.Placement.Matrix)
        last_A_dh1 = mat_to_numpy(last_lcs.Placement.Matrix)
        o_A_current = mat_to_numpy(current_body.Placement.Matrix)
        current_A_dh2 = mat_to_numpy(current_lcs.Placement.Matrix)
        dh1_A_dh2 = np.linalg.inv(last_A_dh1) @ np.linalg.inv(o_A_last) @ o_A_current @ current_A_dh2
        
        d = dh1_A_dh2[2, 3]
        a = dh1_A_dh2[0, 3]
        alpha = np.arccos(dh1_A_dh2[2,2]) if abs(np.arccos(dh1_A_dh2[2,2]) - np.arcsin(dh1_A_dh2[2,1])) < 1e-6 else -np.arccos(dh1_A_dh2[2,2])
        DH_peremeters.append([f'theta_{i}', d, a, alpha])     

    robot.DHPerameters = DH_peremeters
    DH_peremeters = [[round(float(param), 4) if isinstance(param, float) else param for param in dh] for dh in DH_peremeters]
    for dh in DH_peremeters:
        print(dh)

def createDHCoordinateSystems():
    robot = get_robot()
    doc = FreeCAD.ActiveDocument
    empty_body = doc.addObject('PartDesign::Body', 'DH_coordinate_systems')
    doc.recompute()
    dh_coordinate_systems = []
    for i in range(len(robot.DHPerameters)+1):
        lcs_dh = doc.addObject( 'PartDesign::CoordinateSystem', f'DH_{i}' ) 
        empty_body.addObject(lcs_dh)
        dh_coordinate_systems.append(lcs_dh)
    robot.DHCoordinateSystems = dh_coordinate_systems


def updateDHTransformations():
    robot = get_robot()
    last = None
    trans = [ mat_to_numpy(robot.Links[0].Placement.Matrix) @ mat_to_numpy(robot.DHCoordinateSystems[0].Placement.Matrix)]
    for dh_perameters, theta in zip(robot.DHPerameters, robot.Angles):
        _ , d, a, alpha = dh_perameters
        theta = np.deg2rad(theta)
        dh_trans = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                             [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                             [0, np.sin(alpha), np.cos(alpha), d],
                             [0, 0, 0, 1]])
        last = last @ dh_trans if last is not None else dh_trans
        trans.append(last)
    robot.DHTransformations = trans

def positionDHCoordinateSystems():
    robot = get_robot()
    for transformation, dh_coordinate in zip(robot.DHTransformations, robot.DHCoordinateSystems):
        dh_coordinate.Placement.Matrix = numpy_to_mat(transformation)


        
    






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


