import FreeCAD
import FreeCADGui
import os
from main_utils import get_robot


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












# Add to positioning_functions.py
def solve_ik(target_pos, max_iterations=100, tolerance=0.5, damping=0.1):

    print (f"Attemping to solve IK for target position: {target_pos}")
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