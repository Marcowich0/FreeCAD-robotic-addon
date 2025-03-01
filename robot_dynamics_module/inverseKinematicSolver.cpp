/*
 * ik_solver.cpp
 *
 * Implements functions for computing:
 *  - Cumulative DH transformation matrices
 *  - The 6xn end-effector Jacobian
 *  - An inverse kinematics solver (solveIK) using damped least squares.
 *
 * The DH parameters are provided as an Eigen::MatrixXd where each row is:
 * [theta_offset, d, a, alpha]
 */
 #define _USE_MATH_DEFINES
 #include <pybind11/stl.h>
 #include <pybind11/pybind11.h>
 #include <pybind11/eigen.h>
 #include <Eigen/Dense>
 #include <vector>
 #include <cmath>
 
 namespace py = pybind11;
 
 // -----------------------------------------------------------------------------
 // getDHTransformations
 //
 // Computes cumulative transformation matrices for each joint given the joint
 // angles q and DH parameters. Returns a vector of 4x4 matrices where each matrix
 // is the transformation from the base frame up to that joint.
 std::vector<Eigen::Matrix4d> getDHTransformations(const Eigen::VectorXd &q,
    const Eigen::MatrixXd &DHparams)
{
Eigen::Index n = q.size();
std::vector<Eigen::Matrix4d> transforms;
transforms.reserve(n+1);
Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
transforms.push_back(T);

for (Eigen::Index i = 0; i < n; ++i) {
double theta_offset = DHparams(i, 0);  // offset for theta
double theta = q(i) + theta_offset;      // effective joint angle
double d     = DHparams(i, 1);
double a     = DHparams(i, 2);
double alpha = DHparams(i, 3);

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
 // The Jacobian is built as follows:
 //   - The linear component for joint i: Jv = Zi x (On - Oi)
 //     where Zi is the z-axis of the i-th frame (or base if i==0) and Oi is the origin.
 //   - The angular component is simply the z-axis Zi.
 // The end-effector position is taken from the last transformation.
 Eigen::MatrixXd getJacobian(const Eigen::VectorXd &q,
                             const Eigen::MatrixXd &DHparams)
 {
     auto T_arr = getDHTransformations(q, DHparams);
     Eigen::Index n = q.size();
     // End-effector position from the last transformation.
     Eigen::Vector3d On = T_arr.back().block(0, 3, 3, 1);
     Eigen::MatrixXd J(6, n);
 
     for (Eigen::Index i = 1; i <= n; ++i) {
         Eigen::Vector3d Zi, Oi;
         Zi = T_arr[i - 1].block(0, 2, 3, 1);
         Oi = T_arr[i - 1].block(0, 3, 3, 1);
         Eigen::Vector3d Jv = Zi.cross(On - Oi);
         Eigen::Vector3d Jw = Zi;
         J.block(0, i-1, 3, 1) = Jv;
         J.block(3, i-1, 3, 1) = Jw;
     }
     return J;
 }
 
 // -----------------------------------------------------------------------------
 // solveIK
 //
 // Solves the inverse kinematics problem using a damped least-squares method.
 // Parameters:
 //   q                - initial joint angles (in radians)
 //   target_pos       - desired end-effector position (3x1 vector)
 //   target_dir       - desired end-effector direction (z-axis, normalized)
 //   DHparams         - DH parameters matrix (each row: [theta_offset, d, a, alpha])
 //   max_iterations   - maximum number of iterations (default: 100)
 //   tolerance        - error tolerance (default: 0.1)
 //   damping          - damping factor for singularity handling (default: 0.1)
 //   orientation_weight - weight factor for the orientation error (default: 1.0)
 //
 // Returns a pair: (bool converged, Eigen::VectorXd final joint angles).
 // If converged, the joint angles are returned in degrees.
 std::pair<bool, Eigen::VectorXd> solveIK(Eigen::VectorXd q,
    const Eigen::Vector3d &target_pos,
    const Eigen::Vector3d &target_dir,
    const Eigen::MatrixXd &DHparams,
    int max_iterations = 100,
    double tolerance = 0.001, // Adjusted tolerance
    double damping = 0.01, // Adjusted damping
    double orientation_weight = 1.0) {
bool target_active = std::abs(target_dir.norm() - 1.0) < 1e-4;

for (int iter = 0; iter < max_iterations; ++iter) {
auto T_arr = getDHTransformations(q, DHparams);
Eigen::Vector4d current_pos_hom = T_arr.back() * Eigen::Vector4d(0, 0, 0, 1);
Eigen::Vector3d current_pos = current_pos_hom.head();
Eigen::Vector3d delta_x_position = target_pos - current_pos;

double position_error = delta_x_position.norm();
double orientation_error = 0.0;
Eigen::VectorXd delta_x;

if (target_active) {
Eigen::Vector3d current_dir = T_arr.back().block(0, 2, 3, 1);
Eigen::Vector3d orientation_error_vec = current_dir.cross(target_dir);
Eigen::Vector3d delta_x_orientation = orientation_weight * orientation_error_vec;

delta_x.resize(6);
delta_x.head(3) = delta_x_position;
delta_x.tail(3) = delta_x_orientation;
orientation_error = delta_x_orientation.norm();
} else {
delta_x = delta_x_position;
}

double total_error = delta_x.norm();
if (total_error < tolerance) {
return std::make_pair(true, q);
}

Eigen::MatrixXd J_full = getJacobian(q, DHparams);
Eigen::MatrixXd J;
if (target_active)
J = J_full;
else
J = J_full.topRows(3);

Eigen::MatrixXd JJt = J * J.transpose();
Eigen::MatrixXd damping_matrix = (damping * damping) * Eigen::MatrixXd::Identity(JJt.rows(), JJt.rows());
Eigen::MatrixXd J_pseudo = J.transpose() * (JJt + damping_matrix).inverse();

Eigen::VectorXd delta_theta = J_pseudo * delta_x;
q = q + delta_theta;
}

return std::make_pair(false, q);
}
 
 // -----------------------------------------------------------------------------
 // Pybind11 module definition
 // Expose getDHTransformations, getJacobian, and solveIK to Python.
 PYBIND11_MODULE(inverse_kinematics_cpp, m) {
    m.doc() = "Inverse kinematics helpers: compute DH transformations, Jacobian, and solve IK.";
    m.def("getDHTransformations", &getDHTransformations,
          py::arg("q"), py::arg("DHparams"),
          "Return cumulative DH transformation matrices given joint angles q and DH parameters.");
    m.def("getJacobian", &getJacobian,
          py::arg("q"), py::arg("DHparams"),
          "Return the 6xn end-effector Jacobian given joint angles q and DH parameters.");
    m.def("solveIK", &solveIK,
          py::arg("q"),
          py::arg("target_pos"),
          py::arg("target_dir"),
          py::arg("DHparams"),
          py::arg("max_iterations") = 100,
          py::arg("tolerance") = 0.1,
          py::arg("damping") = 0.1,
          py::arg("orientation_weight") = 1.0,
          "Solve inverse kinematics to reach the target position and (optionally) align with the target direction.\n"
          "Returns a tuple: (converged, joint_angles). If converged is true, joint_angles are in degrees.");
}