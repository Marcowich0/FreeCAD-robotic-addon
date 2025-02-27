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
    
def np_translation(x, y, z):
    return np.array([[1, 0, 0, x],
                     [0, 1, 0, y],
                     [0, 0, 1, z],
                     [0, 0, 0, 1]])


def vec_to_numpy(v, n=3):
    # Assumes v has attributes x, y, z.
    if n == 3:
        return np.array([v.x, v.y, v.z])
    elif n == 4:
        return np.array([v.x, v.y, v.z, 1])


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
    

def displayMatrix(matrix):
    """
    Prints a numeric matrix with aligned columns, rounding to 3 decimals
    and replacing near-zero values with 0.

    Accepts:
    - list/tuple of numbers (1D or 2D)
    - np.ndarray (1D or 2D)
    - jax.numpy array (1D or 2D), if jax is installed

    Example:
        displayMatrix([1.234567, 0.0000000001, 3.14159])
        displayMatrix([[1.234567, 2.71828], [3.14159, 0.00000001]])
    """

    print("---------------------------------")

    # If it's a JAX array, convert to a NumPy array
    # (only do this if jax is available in your environment)
    try:
        import jax.numpy as jnp
        if isinstance(matrix, jnp.ndarray):
            matrix = np.array(matrix)
    except ImportError:
        pass  # jax not installed, no conversion needed

    # Convert Python lists/tuples (or any other type) to NumPy array
    if not isinstance(matrix, np.ndarray):
        matrix = np.array(matrix)

    # Ensure matrix is at least 2D (if 1D, reshape to (1, -1))
    if matrix.ndim == 1:
        matrix = matrix.reshape(1, -1)

    # Convert values close to zero to 0 and round to 3 decimals
    # We'll create a float copy first, to avoid issues with object dtypes
    matrix = matrix.astype(float)
    matrix = np.where(np.abs(matrix) < 1e-10, 0, np.round(matrix, 3))

    # Convert all values to strings with 3 decimal places
    string_matrix = [[f"{val:.3f}" for val in row] for row in matrix]

    # Determine column widths by taking the max string length in each column
    col_widths = [max(len(item) for item in col) for col in zip(*string_matrix)]

    # Print each row with aligned columns
    for row in string_matrix:
        formatted_row = "  ".join(val.rjust(width) for val, width in zip(row, col_widths))
        print(formatted_row)

    print("---------------------------------")



