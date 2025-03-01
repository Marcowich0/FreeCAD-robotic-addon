
import numpy as np
def solve_ik(q, target_pos, target_dir, max_iterations=100, tolerance=0.1, damping=0.1, orientation_weight=1.0, collision = True):
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
    target_active = abs(target_dir.Length - 1) < 1e-4
        
    for iteration in range(max_iterations):
        
        T_arr = getDHTransformations(q)
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
            return [*np.rad2deg(q)]
        
        # Calculate Jacobian at current position
        J_full = getJacobian(q, SI = False)
        
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


    return False