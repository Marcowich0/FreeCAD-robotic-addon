import FreeCAD
import FreeCADGui
import os
from main_utils import get_robot, vec_to_numpy, displayMatrix
from secondary_utils import checkCollision
from forward_kinematics import getDHTransformations, getJacobian
import numpy as np
import math



import inverse_kinematics_cpp
class ToTargetPointCommand:
    """Command to solve inverse kinematics for a target position."""
    def __init__(self):
        pass

    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'toPoint.svg'),
            'MenuText': 'Solve Inverse Kinematics',
            'ToolTip': 'Solve joint angles to reach the selected target point'
        }

    def Activated(self):
        global_pos = np.array(self.get_target_position())
        robot = get_robot()
        q = np.deg2rad(robot.Angles)
        DHperameters = np.array(robot.DHPerameters)
        DHperameters[:,0] = 0
        DHperameters = DHperameters.astype(float)
        DHperameters[:,1:3] /= 1000

        displayMatrix(q)
        displayMatrix(getDHTransformations(q, SI = True)[-1])
        displayMatrix(inverse_kinematics_cpp.getDHTransformations(q, DHperameters)[-1])

        displayMatrix(getJacobian(q, SI = True))
        displayMatrix(inverse_kinematics_cpp.getJacobian(q, DHperameters))

        print(f"type of inputs, q: {type(q)}, global_pos: {type(global_pos)}, DHperameters: {type(DHperameters)}")
        displayMatrix(global_pos)
        displayMatrix(DHperameters)

        converged, q = inverse_kinematics_cpp.solveIK(q, global_pos/1000, vec_to_numpy(robot.EndEffectorOrientation), DHperameters)
        robot.Angles =  np.rad2deg(q).tolist()
        displayMatrix(np.rad2deg(q))

    def IsActive(self):
        return self.is_target_selected() and get_robot() is not None

    def get_target_position(self):
        """
        Returns the global 3D coordinates of the selected vertex—the exact 
        same coordinates you would see in the Measure tool.
        """
        sel = FreeCADGui.Selection.getSelectionEx()
        if sel:
            sobj = sel[0]
            # The selection object stores the 3D pick positions in PickedPoints.
            # If you've selected a vertex, then sobj.PickedPoints[0] is the global position.
            if sobj.PickedPoints:
                return sobj.PickedPoints[0]
        return None

    def is_target_selected(self):
        sel = FreeCADGui.Selection.getSelectionEx()
        return bool(sel 
                    and sel[0].SubObjects
                    and sel[0].SubObjects[0].ShapeType == "Vertex" 
                    and sel[0].PickedPoints)

FreeCADGui.addCommand('ToTargetPointCommand', ToTargetPointCommand())


def solve_ik(q, target_pos, target_dir, DHperameters):
    print(f"Received q: {q}")
    robot = get_robot()
    q = np.deg2rad(robot.Angles)
    for i in range(300):
        converged, q = inverse_kinematics_cpp.solveIK(q, target_pos, target_dir, DHperameters)
        robot.Angles =  np.rad2deg(q).tolist()
        FreeCAD.ActiveDocument.recompute()
        if checkCollision():
            q = [np.random.uniform(-np.pi, np.pi) for _ in range(len(q))]
        else:
            print(f"Returning q: {q}")
            return q
    




def solve_ik_old(q, target_pos, target_dir):
    """
    Solves inverse kinematics to reach target position and optionally align with target direction.
    
    Args:
        target_pos (FreeCAD.Vector): Target position in global coordinates
        target_dir (FreeCAD.Vector, optional): Target direction vector (z-axis alignment). Defaults to None.
        max_iterations (int): Maximum number of iterations (default: 1000)
        tolerance (float): Combined position/orientation error tolerance (default: 0.5)
        damping (float): Damping factor for singularity handling (default: 0.1)
        orientation_weight (float): Weight for orientation error relative to position (default: 1.0)
        
    Returns:
        bool: True if converged, False if failed
    """

    max_iterations=300
    tolerance=0.1
    damping=0.1
    orientation_weight=1.0

    target_active = abs(np.linalg.norm(target_dir) - 1) < 1e-4
        
    for iteration in range(max_iterations):
        
        T_arr = getDHTransformations(q, SI=True)
        current_pos = (T_arr[-1] @ np.array([0, 0, 0, 1]))[:3]

        delta_x_position = target_pos - current_pos

        # Initialize error components
        position_error = np.linalg.norm(delta_x_position)
        orientation_error = 0.0

        # Calculate orientation components if target_dir is specified
        if target_active:
            
            current_dir = T_arr[-1][:3, 2]
            
            # Calculate orientation error using cross product
            orientation_error_vec = np.cross(current_dir, target_dir)
            delta_x_orientation = orientation_weight * orientation_error_vec
            
            # Combine position and orientation errors
            delta_x = np.concatenate([delta_x_position, delta_x_orientation])
            orientation_error = np.linalg.norm(delta_x_orientation)
        else:
            delta_x = delta_x_position

        # Calculate total error
        total_error = np.linalg.norm(delta_x)
        if total_error < tolerance:
            return q
        
        # Calculate Jacobian at current position
        J_full = getJacobian(q, SI = True)
        
        # Select appropriate Jacobian based on orientation solving
        J = J_full if target_active else J_full[:3, :]

        # Damped least squares inversion
        m = J.shape[0]
        J_T = J.T
        JJT = J @ J_T
        damping_matrix = (damping**2) * np.eye(m)
        J_pseudo = J_T @ np.linalg.inv(JJT + damping_matrix)
        
        # Calculate angle changes
        delta_theta_rad = J_pseudo @ delta_x
        q = q + delta_theta_rad.flatten()

    FreeCAD.Console.PrintWarning(
        f"IK failed to converge after {max_iterations} iterations\n"
        f"Final position error: {position_error:.2f} mm"
        + (f", orientation error: {orientation_error:.4f} rad" if target_dir.any() else "") + "\n"
    )

    return False

