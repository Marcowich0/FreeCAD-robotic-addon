import FreeCAD
import FreeCADGui
import os
from main_utils import get_robot
import numpy as np
import math

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
        global_pos = self.get_target_position()
        if global_pos:
            solve_ik(global_pos)

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







def solve_ik(target_pos, max_iterations=1000, tolerance=0.05, damping=0.1, orientation_weight=1.0):
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
    robot = get_robot()
    target_dir = robot.EndEffectorOrientation
    target_active = abs(target_dir.Length - 1) < 1e-4
    
    for iteration in range(max_iterations):
        current_pos = robot.EndEffectorGlobal
        delta_pos = target_pos - current_pos
        delta_x_position = np.array([delta_pos.x, delta_pos.y, delta_pos.z])

        # Initialize error components
        position_error = np.linalg.norm(delta_x_position)
        orientation_error = 0.0

        # Calculate orientation components if target_dir is specified
        if target_active:
            # Get current end effector orientation (z-axis)
            o_A_b = robot.Bodies[-1].Placement.Rotation
            b_A_lc = robot.BodyJointCoordinateSystems[-1].Placement.Rotation
            current_rot = o_A_b.multiply(b_A_lc)
            current_dir = current_rot.multVec(FreeCAD.Vector(0, 0, 1))
            current_dir.normalize()  # Normalize in-place

            
            # Calculate orientation error using cross product
            orientation_error_vec = current_dir.cross(target_dir)
            delta_x_orientation = orientation_weight * np.array([
                orientation_error_vec.x,
                orientation_error_vec.y,
                orientation_error_vec.z
            ])
            
            # Combine position and orientation errors
            delta_x = np.concatenate([delta_x_position, delta_x_orientation])
            orientation_error = np.linalg.norm(delta_x_orientation)
        else:
            delta_x = delta_x_position

        # Calculate total error
        total_error = np.linalg.norm(delta_x)
        if total_error < tolerance:
            FreeCAD.Console.PrintMessage(
                f"IK converged after {iteration+1} iterations\n"
                f"Position error: {position_error:.2f} mm"
                + (f", Orientation error: {orientation_error:.4f} rad" if target_dir else "") + "\n"
            )
            return True

        # Convert current angles to radians for calculations
        current_angles_rad = np.array([math.radians(a) for a in robot.Angles])
        
        # Calculate Jacobian at current position
        J_full = robot.NumpyJacobian(*current_angles_rad)
        
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
        new_angles_rad = current_angles_rad + delta_theta_rad.flatten()
        
        # Convert back to degrees and normalize
        new_angles_deg = [math.degrees(a) % 360 for a in new_angles_rad]
        new_angles_deg = [(a + 180) % 360 - 180 for a in new_angles_deg]
        
        # Update robot angles
        robot.Angles = new_angles_deg
        FreeCAD.ActiveDocument.recompute()

    FreeCAD.Console.PrintWarning(
        f"IK failed to converge after {max_iterations} iterations\n"
        f"Final position error: {position_error:.2f} mm"
        + (f", orientation error: {orientation_error:.4f} rad" if target_dir else "") + "\n"
    )
    return False

