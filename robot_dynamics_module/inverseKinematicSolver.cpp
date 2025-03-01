/*
 * ik_solver.cpp
 *
 * Implements functions for computing:
 *  - Cumulative DH transformation matrices
 *  - The 6xn end-effector Jacobian
 *  - An inverse kinematics solver (solveIK) using damped least squares.
 *
 * The DH parameters are provided as an Eigen::MatrixXd where each row is:
 * [theta_offset, d, a, alpha].
 */

 #define _USE_MATH_DEFINES
 #include <pybind11/stl.h>
 #include <pybind11/pybind11.h>
 #include <pybind11/eigen.h>
 #include <Eigen/Dense>
 #include <vector>
 #include <cmath>
 #include <iostream>
 
 namespace py = pybind11;
 
 // -----------------------------------------------------------------------------
 // getDHTransformations
 //
 // Computes cumulative transformation matrices for each joint given the joint
 // angles q and DH parameters. Returns a vector of 4x4 matrices, where each matrix
 // is the transform from the base frame up to that joint frame.
 std::vector<Eigen::Matrix4d> getDHTransformations(const Eigen::VectorXd &q,
                                                   const Eigen::MatrixXd &DHparams)
 {
     Eigen::Index n = q.size();
     std::vector<Eigen::Matrix4d> transforms;
     transforms.reserve(n + 1);
 
     // Start with identity for the "base" frame
     Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
     transforms.push_back(T);
 
     for (Eigen::Index i = 0; i < n; ++i) {
         double theta_offset = DHparams(i, 0);  // offset for theta
         double theta = q(i) + theta_offset;    // effective joint angle
         double d     = DHparams(i, 1);
         double a     = DHparams(i, 2);
         double alpha = DHparams(i, 3);
 
         // Single DH transform for this joint
         Eigen::Matrix4d A;
         A <<  std::cos(theta), -std::sin(theta)*std::cos(alpha),  std::sin(theta)*std::sin(alpha),  a*std::cos(theta),
               std::sin(theta),  std::cos(theta)*std::cos(alpha), -std::cos(theta)*std::sin(alpha),  a*std::sin(theta),
               0,                std::sin(alpha),                  std::cos(alpha),                  d,
               0,                0,                                0,                                1;
 
         T = T * A;
         transforms.push_back(T);
     }
     return transforms;
 }
 
 // -----------------------------------------------------------------------------
 // getJacobian
 //
 // Computes the 6xn end-effector Jacobian given joint angles q and DH parameters.
 // The Jacobian layout:
 //   - Linear (rows 0..2): Zi x (On - Oi)
 //   - Angular (rows 3..5): Zi
 // where Zi is the z-axis of frame i, Oi is the origin of frame i, and On is
 // the end-effector origin.
 Eigen::MatrixXd getJacobian(const Eigen::VectorXd &q,
                             const Eigen::MatrixXd &DHparams)
 {
     auto T_arr = getDHTransformations(q, DHparams);
     Eigen::Index n = q.size();
 
     // End-effector position from the last transform
     Eigen::Vector3d On = T_arr.back().block(0, 3, 3, 1);
 
     Eigen::MatrixXd J(6, n);
 
     for (Eigen::Index i = 1; i <= n; ++i) {
         // Zi is the z-axis of the (i-1)-th frame
         Eigen::Vector3d Zi = T_arr[i - 1].block(0, 2, 3, 1);
         // Oi is the origin of the (i-1)-th frame
         Eigen::Vector3d Oi = T_arr[i - 1].block(0, 3, 3, 1);
 
         // Linear part
         Eigen::Vector3d Jv = Zi.cross(On - Oi);
         // Angular part
         Eigen::Vector3d Jw = Zi;
 
         J.block(0, i - 1, 3, 1) = Jv;
         J.block(3, i - 1, 3, 1) = Jw;
     }
 
     return J;
 }
 
 // -----------------------------------------------------------------------------
 // solveIK
 //
 // Solves the inverse kinematics problem using a damped least-squares method
 // to move the manipulator's end-effector to (target_pos) and align its Z-axis
 // with (target_dir), if target_dir is approximately unit length.
 //
 // Hard-coded parameters:
 //   - max_iterations      = 100
 //   - tolerance           = 0.001
 //   - damping             = 0.01
 //   - orientation_weight  = 1.0
 //
 // Returns (converged, final_q). Angles are in radians.
 std::pair<bool, Eigen::VectorXd> solveIK(Eigen::VectorXd q,
    const Eigen::Vector3d &target_pos,
    const Eigen::Vector3d &target_dir,
    const Eigen::MatrixXd &DHparams)
{
// Hard-coded solver parameters
const int max_iterations = 300;
const double tolerance   = 0.1;
const double damping     = 0.1;   // Damping factor
const double orientation_weight = 1.0;
// Check if the provided target_dir is near unit length
bool target_active = (std::fabs(target_dir.norm() - 1.0) < 1e-4);

bool converged = false;

for (int iter = 0; iter < max_iterations; ++iter) {
// Get forward kinematics
auto T_arr = getDHTransformations(q, DHparams);

// Current end-effector position
Eigen::Vector3d current_pos = T_arr.back().block(0, 3, 3, 1);

// Position error
Eigen::Vector3d delta_x_position = target_pos - current_pos;
double position_error = delta_x_position.norm();

// Build the task-space error vector
Eigen::VectorXd delta_x;
double orientation_error = 0.0;

if (target_active) {
// Current end-effector Z-axis
Eigen::Vector3d current_dir = T_arr.back().block(0, 2, 3, 1);

// Orientation error via cross product
Eigen::Vector3d orientation_error_vec = current_dir.cross(target_dir);
Eigen::Vector3d delta_x_orientation = orientation_weight * orientation_error_vec;
orientation_error = delta_x_orientation.norm();

// Combine position & orientation
delta_x.resize(6);
delta_x << delta_x_position, delta_x_orientation;
}
else {
// Only position
delta_x.resize(3);
delta_x = delta_x_position;
}

// Check total error
double total_error = delta_x.norm();
if (total_error < tolerance) {
converged = true;
break;
}

// Compute Jacobian
Eigen::MatrixXd J_full = getJacobian(q, DHparams);

// If orientation is active, use full 6xN, otherwise use top 3 rows
Eigen::MatrixXd J;
if (target_active) {
J = J_full;
} else {
// Just take the linear part (rows 0..2)
J = J_full.block(0, 0, 3, J_full.cols());
}

// Damped least squares: J_pseudo = J^T * (J*J^T + λ^2 I)^{-1}
const int m = J.rows();
Eigen::MatrixXd JJT = J * J.transpose();
Eigen::MatrixXd damping_matrix = (damping * damping)
    * Eigen::MatrixXd::Identity(m, m);
Eigen::MatrixXd invTerm = (JJT + damping_matrix).inverse();

Eigen::MatrixXd J_pseudo = J.transpose() * invTerm;

// Update joint angles
Eigen::VectorXd delta_theta = J_pseudo * delta_x;
q += delta_theta;
}

return std::make_pair(converged, q);
}
 
 // -----------------------------------------------------------------------------
 // Pybind11 module definition
 //
 // Now we only expose the arguments that remain: q, target_pos, target_dir, DHparams.
 //
 // The solver is using hard-coded iteration, damping, tolerance, orientation weight.
 PYBIND11_MODULE(inverse_kinematics_cpp, m) {
     m.doc() = "Inverse kinematics helpers for a manipulator using DH parameters and a damped-LS IK solver.";
 
     m.def("getDHTransformations", &getDHTransformations,
           py::arg("q"), py::arg("DHparams"),
           "Return cumulative DH transformation matrices given joint angles q and DH parameters.");
 
     m.def("getJacobian", &getJacobian,
           py::arg("q"), py::arg("DHparams"),
           "Return the 6xn end-effector Jacobian given joint angles q and DH parameters.");
 
     // No additional arguments for the solver; they are hard-coded inside the function.
     m.def("solveIK", &solveIK,
           py::arg("q"),
           py::arg("target_pos"),
           py::arg("target_dir"),
           py::arg("DHparams"),
           "Solve inverse kinematics using a damped LS approach with fixed iteration/damping/tolerance.\n"
           "Returns (converged, final_angles_in_radians).");
 }
 