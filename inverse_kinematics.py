import FreeCAD
import FreeCADGui
import os
from main_utils import get_robot

# Add to positioning_functions.py
def solve_ik(target_pos, max_iterations=50, tolerance=1.0, damping=0.1):
    """
    Solves inverse kinematics to reach target position with minimal joint movement.
    
    Args:
        target_pos (FreeCAD.Vector): Target position in global coordinates
        max_iterations (int): Maximum number of iterations (default: 50)
        tolerance (float): Position error tolerance in mm (default: 1.0)
        damping (float): Damping factor for singularity handling (default: 0.1)
        
    Returns:
        bool: True if converged, False if failed
    """
    robot = get_robot()
    if not robot:
        FreeCAD.Console.PrintError("No robot found\n")
        return False

    import numpy as np
    import math
    
    for iteration in range(max_iterations):
        current_pos = robot.EndEffectorGlobal
        delta_x = np.array([
            target_pos.x - current_pos.x,
            target_pos.y - current_pos.y,
            target_pos.z - current_pos.z
        ])
        
        error = np.linalg.norm(delta_x)
        if error < tolerance:
            FreeCAD.Console.PrintMessage(f"IK converged after {iteration+1} iterations\n")
            return True

        # Convert current angles to radians for calculations
        current_angles_rad = np.array([math.radians(a) for a in robot.Angles])
        
        # Calculate Jacobian at current position (use only positional part)
        J_full = robot.NumpyJacobian(*current_angles_rad)
        J_position = J_full[:3, :]  # Extract first 3 rows (positional components)
        
        # Damped least squares inversion
        J_T = J_position.T
        JJT = J_position @ J_T
        damping_matrix = damping**2 * np.eye(3)
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

    FreeCAD.Console.PrintWarning(f"IK failed to converge after {max_iterations} iterations. Final error: {error:.2f} mm\n")
    return False