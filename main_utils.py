import FreeCAD
import FreeCADGui
import numpy as np

def get_robot():
    for obj in FreeCAD.ActiveDocument.Objects:
        if hasattr(obj, 'Type') and obj.Type == 'Robot':
            return obj
    return None

def mat_to_numpy(matrix):
    n = int(np.sqrt(len(matrix.A)))
    return np.array(matrix.A).reshape(n,n)

def numpy_to_mat(matrix):
    return FreeCAD.Matrix(*matrix.flatten())

def numpy_to_rotation(matrix):
    return FreeCAD.Rotation(matrix)

def np_rotation(angle, axis):
    if axis == 'x':
        return np.array([[1, 0, 0, 0],
                         [0, np.cos(angle), -np.sin(angle), 0],
                         [0, np.sin(angle), np.cos(angle), 0],
                         [0, 0, 0, 1]])
    elif axis == 'y':
        return np.array([[np.cos(angle), 0, np.sin(angle), 0],
                         [0, 1, 0, 0],
                         [-np.sin(angle), 0, np.cos(angle), 0],
                         [0, 0, 0, 1]])
    elif axis == 'z':
        return np.array([[np.cos(angle), -np.sin(angle), 0, 0],
                         [np.sin(angle), np.cos(angle), 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
    

def vec_to_numpy(v):
    # Assumes v has attributes x, y, z.
    return np.array([v.x, v.y, v.z])


def updateGlobalEndEffector():
    robot = get_robot()
    if robot.NumpyTransformations:
        T_last = robot.NumpyTransformations[-1](*[float(angle/180*np.pi) for angle in robot.Angles])
        robot.EndEffectorGlobal = (float(T_last[0, 3]), float(T_last[1, 3]), float(T_last[2, 3]))




def currentSelectionType():
    selection = FreeCADGui.Selection.getSelectionEx()
    if selection and selection[0].SubElementNames:
        sel = selection[0]
        subelement = sel.SubElementNames[0]
        if subelement.startswith('Edge'):
            return 'Edge'
        elif subelement.startswith('Face'):
            return 'Face'
        elif subelement.startswith('Vertex'):
            return 'Vertex'
        
    try:
        return sel.Type
    except:
        return None