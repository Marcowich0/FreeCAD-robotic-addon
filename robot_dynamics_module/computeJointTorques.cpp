/*
 * computeJointTorques.cpp
 *
 * Complete single-file implementation for computing robot joint torques 
 * using DH-based dynamics. The DH parameters are passed as a NumPy matrix 
 * (each row: [theta_offset, d, a, alpha]) and are used to compute the 
 * cumulative transforms.
 *
 * This file uses Eigen for matrix operations and pybind11 to expose the function 
 * to Python.
 */

 #include <pybind11/pybind11.h>
 #include <pybind11/eigen.h>
 #include <pybind11/numpy.h>
 #include <Eigen/Dense>
 #include <vector>
 #include <cmath>
 #include <functional>
 #include <stdexcept>

 
 
 namespace py = pybind11;
 
 // -----------------------------------------------------------------------------
 // Helper: Numerical partial derivative for matrix-valued functions.
 // f: R^n -> R^(rows x cols)
 Eigen::MatrixXd partialDerivativeMatrix(
     const std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>& f,
     const Eigen::VectorXd& x,
     int index,
     double eps = 1e-6)
 {
     Eigen::VectorXd x_plus  = x;
     Eigen::VectorXd x_minus = x;
     x_plus(index)  += eps;
     x_minus(index) -= eps;
     Eigen::MatrixXd f_plus  = f(x_plus);
     Eigen::MatrixXd f_minus = f(x_minus);
     return (f_plus - f_minus) / (2.0 * eps);
 }
 
 // -----------------------------------------------------------------------------
 // Helper: Numerical partial derivative for vector-valued functions.
 // f: R^n -> R^m.
 Eigen::VectorXd partialDerivativeVector(
     const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& f,
     const Eigen::VectorXd& x,
     int index,
     double eps = 1e-6)
 {
     Eigen::VectorXd x_plus  = x;
     Eigen::VectorXd x_minus = x;
     x_plus(index)  += eps;
     x_minus(index) -= eps;
     Eigen::VectorXd f_plus  = f(x_plus);
     Eigen::VectorXd f_minus = f(x_minus);
     return (f_plus - f_minus) / (2.0 * eps);
 }
 
 // -----------------------------------------------------------------------------
 // getDHTransformations
 // Computes the cumulative transformation matrices given joint angles q and 
 // DH parameters (each row: [theta_offset, d, a, alpha]). 
 std::vector<Eigen::Matrix4d> getDHTransformations(const Eigen::VectorXd& q,
                                                   const Eigen::MatrixXd& DHparams)
 {
     size_t n = static_cast<size_t>(q.size());
     std::vector<Eigen::Matrix4d> transforms;
     transforms.reserve(n);
     Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
     
     for (size_t i = 0; i < n; ++i) {
         double theta_offset = DHparams(i, 0);   // offset for theta
         double theta = q(i) + theta_offset;       // effective joint angle
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
 // getJacobianCenter
 // For each link, compute the 6 x n Jacobian for the link's center of mass 
 // using the transforms and the given COM positions (in the link's local frame).
 std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>>
 getJacobianCenter(const Eigen::VectorXd& q,
                   const std::vector<Eigen::Matrix4d>& transforms,
                   const std::vector<Eigen::Vector3d>& comPositions)
 {
     size_t n = static_cast<size_t>(q.size());
     std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> jacobians;
     jacobians.reserve(n);
     
     const Eigen::Vector3d baseZ(0, 0, 1);
     const Eigen::Vector3d baseOrigin(0, 0, 0);
     
     for (size_t i = 0; i < n; ++i) {
         // Transform COM to global coordinates.
         Eigen::Vector4d comLocalH(comPositions[i](0), comPositions[i](1), comPositions[i](2), 1.0);
         Eigen::Vector4d comGlobalH = transforms[i] * comLocalH;
         Eigen::Vector3d comGlobal  = comGlobalH.head<3>();
         
         // Initialize a 6xn Jacobian for link i.
         Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, n);
         J.setZero();
         
         for (size_t j = 0; j < n; ++j) {
             Eigen::Vector3d zAxis, origin_j;
             if (j == 0) {
                 zAxis    = baseZ;
                 origin_j = baseOrigin;
             } else {
                 zAxis    = transforms[j - 1].block<3,1>(0, 2);
                 origin_j = transforms[j - 1].block<3,1>(0, 3);
             }
             
             if (j <= i) {
                 Eigen::Vector3d r = comGlobal - origin_j;
                 Eigen::Vector3d Jv = zAxis.cross(r);
                 Eigen::Vector3d Jw = zAxis;
                 J.block<3,1>(0, j) = Jv;
                 J.block<3,1>(3, j) = Jw;
             }
         }
         jacobians.push_back(J);
     }
     return jacobians;
 }
 
 // -----------------------------------------------------------------------------
 // computeD
 // Computes the inertia matrix D(q) = Σ [ m_i * (Jv_i^T Jv_i) + (Jw_i^T I_global_i Jw_i) ]
 // DHparams are passed to get the correct transforms.
 Eigen::MatrixXd computeD(const Eigen::VectorXd& q,
                          const std::vector<double>& masses,
                          const std::vector<Eigen::Matrix3d>& inertiaMatrices,
                          const std::vector<Eigen::Vector3d>& comPositions,
                          const Eigen::MatrixXd& DHparams)
 {
     size_t n = static_cast<size_t>(q.size());
     Eigen::MatrixXd D = Eigen::MatrixXd::Zero(n, n);
     
     std::vector<Eigen::Matrix4d> transforms = getDHTransformations(q, DHparams);
     std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> jacobians = 
         getJacobianCenter(q, transforms, comPositions);
     
     for (size_t i = 0; i < n; ++i) {
         Eigen::Matrix<double, 3, Eigen::Dynamic> Jv_i = jacobians[i].topRows(3);
         Eigen::Matrix<double, 3, Eigen::Dynamic> Jw_i = jacobians[i].bottomRows(3);
         Eigen::Matrix3d R_i = transforms[i].block<3,3>(0, 0);
         Eigen::Matrix3d I_global = R_i * inertiaMatrices[i] * R_i.transpose();
         D += masses[i] * (Jv_i.transpose() * Jv_i)
            + (Jw_i.transpose() * I_global * Jw_i);
     }
     return D;
 }
 
 // -----------------------------------------------------------------------------
 // computeP
 // Computes potential energy P(q) = Σ m_i * g^T * com_global_i.
 double computeP(const Eigen::VectorXd& q,
                 const std::vector<double>& masses,
                 const std::vector<Eigen::Vector3d>& comPositions,
                 const Eigen::MatrixXd& DHparams)
 {
     size_t n = static_cast<size_t>(q.size());
     const Eigen::Vector3d gravity(0, 0, -9.81);
     double P = 0.0;
     std::vector<Eigen::Matrix4d> transforms = getDHTransformations(q, DHparams);
     for (size_t i = 0; i < n; ++i) {
         Eigen::Vector4d comLocalH(comPositions[i](0), comPositions[i](1), comPositions[i](2), 1.0);
         Eigen::Vector4d comGlobalH = transforms[i] * comLocalH;
         Eigen::Vector3d comGlobal  = comGlobalH.head<3>();
         P += masses[i] * gravity.dot(comGlobal);
     }
     return P;
 }
 
 // -----------------------------------------------------------------------------
 // compute_g
 // Computes gravity torque vector g(q) = ∂P/∂q via numeric differentiation.
 Eigen::VectorXd compute_g(const Eigen::VectorXd& q,
                           const std::vector<double>& masses,
                           const std::vector<Eigen::Vector3d>& comPositions,
                           const Eigen::MatrixXd& DHparams)
 {
     size_t n = static_cast<size_t>(q.size());
     Eigen::VectorXd gVec = Eigen::VectorXd::Zero(n);
     
     auto P_func = [&](const Eigen::VectorXd& q_val) {
         double p_val = computeP(q_val, masses, comPositions, DHparams);
         Eigen::VectorXd out(1);
         out(0) = p_val;
         return out;
     };
     
     for (size_t i = 0; i < n; ++i) {
         Eigen::VectorXd grad_i = partialDerivativeVector(P_func, q, static_cast<int>(i));
         gVec(i) = grad_i(0);
     }
     return gVec;
 }
 
 // -----------------------------------------------------------------------------
 // computeC
 // Computes the Coriolis matrix C(q, q_dot) using numerical differentiation of D(q).
 Eigen::MatrixXd computeC(const Eigen::VectorXd& q,
                          const Eigen::VectorXd& q_dot,
                          const std::vector<double>& masses,
                          const std::vector<Eigen::Matrix3d>& inertiaMatrices,
                          const std::vector<Eigen::Vector3d>& comPositions,
                          const Eigen::MatrixXd& DHparams)
 {
     size_t n = static_cast<size_t>(q.size());
     Eigen::MatrixXd Cmat = Eigen::MatrixXd::Zero(n, n);
     
     auto D_func = [&](const Eigen::VectorXd& q_val) {
         return computeD(q_val, masses, inertiaMatrices, comPositions, DHparams);
     };
     
     std::vector<Eigen::MatrixXd> dD_dqi(n);
     for (size_t i = 0; i < n; ++i) {
         dD_dqi[i] = partialDerivativeMatrix(D_func, q, static_cast<int>(i));
     }
     for (size_t k = 0; k < n; ++k) {
         for (size_t j = 0; j < n; ++j) {
             double ckj = 0.0;
             for (size_t i = 0; i < n; ++i) {
                 ckj += 0.5 * ( dD_dqi[i](k, j) + dD_dqi[j](k, i) - dD_dqi[k](i, j) ) * q_dot(i);
             }
             Cmat(k, j) = ckj;
         }
     }
     return Cmat;
 }
 
 // -----------------------------------------------------------------------------
 // computeJointTorques
 // Main function: τ = D(q)*q_ddot + C(q, q_dot)*q_dot + g(q)
 // This function uses internal types (std::vector, Eigen types) for masses, inertiaMatrices, and comPositions.
 Eigen::VectorXd computeJointTorques(const Eigen::VectorXd& q,
                                     const Eigen::VectorXd& q_dot,
                                     const Eigen::VectorXd& q_ddot,
                                     const std::vector<double>& masses,
                                     const std::vector<Eigen::Matrix3d>& inertiaMatrices,
                                     const std::vector<Eigen::Vector3d>& comPositions,
                                     const Eigen::MatrixXd& DHparams)
 {
     Eigen::MatrixXd D_mat = computeD(q, masses, inertiaMatrices, comPositions, DHparams);
     Eigen::MatrixXd C_mat = computeC(q, q_dot, masses, inertiaMatrices, comPositions, DHparams);
     Eigen::VectorXd g_vec = compute_g(q, masses, comPositions, DHparams);
     Eigen::VectorXd tau = D_mat * q_ddot + C_mat * q_dot + g_vec;
     return tau;
 }
 
 // -----------------------------------------------------------------------------
 // computeJointTorquesWrapper
 // This wrapper function accepts all inputs as NumPy arrays (with inertiaMatrices 
 // as a 3D array of shape (n,3,3) and comPositions as a 2D array of shape (n,3))
 // and converts them to the types required by computeJointTorques.
 Eigen::VectorXd computeJointTorquesWrapper(
     const Eigen::VectorXd& q,
     const Eigen::VectorXd& q_dot,
     const Eigen::VectorXd& q_ddot,
     py::array_t<double> masses_array,
     py::array_t<double> inertiaMatrices_array,
     py::array_t<double> comPositions_array,
     const Eigen::MatrixXd& DHparams)
 {
     // Convert masses (expected 1D array)
     auto masses_buf = masses_array.request();
     if(masses_buf.ndim != 1)
         throw std::runtime_error("masses must be a 1D numpy array");
     size_t n = masses_buf.shape[0];
     std::vector<double> masses(n);
     double* masses_ptr = static_cast<double*>(masses_buf.ptr);
     for (size_t i = 0; i < n; i++) {
         masses[i] = masses_ptr[i];
     }
 
     // Convert inertiaMatrices (expected shape: (n, 3, 3))
     auto inertia_buf = inertiaMatrices_array.request();
     if(inertia_buf.ndim != 3)
         throw std::runtime_error("inertiaMatrices must be a 3D numpy array with shape (n,3,3)");
     if(inertia_buf.shape[1] != 3 || inertia_buf.shape[2] != 3)
         throw std::runtime_error("inertiaMatrices must have shape (n,3,3)");
     size_t n_inertia = inertia_buf.shape[0];
     if(n_inertia != n)
         throw std::runtime_error("Number of inertia matrices must equal number of masses");
     std::vector<Eigen::Matrix3d> inertiaMatrices;
     inertiaMatrices.reserve(n);
     double* inertia_ptr = static_cast<double*>(inertia_buf.ptr);
     for (size_t i = 0; i < n; i++) {
         Eigen::Matrix3d I;
         for (size_t r = 0; r < 3; r++) {
             for (size_t c = 0; c < 3; c++) {
                 I(r, c) = inertia_ptr[i * 9 + r * 3 + c];
             }
         }
         inertiaMatrices.push_back(I);
     }
 
     // Convert comPositions (expected shape: (n, 3))
     auto com_buf = comPositions_array.request();
     if(com_buf.ndim != 2)
         throw std::runtime_error("comPositions must be a 2D numpy array with shape (n,3)");
     if(com_buf.shape[1] != 3)
         throw std::runtime_error("comPositions must have shape (n,3)");
     size_t n_com = com_buf.shape[0];
     if(n_com != n)
         throw std::runtime_error("Number of comPositions must equal number of masses");
     std::vector<Eigen::Vector3d> comPositions;
     comPositions.reserve(n);
     double* com_ptr = static_cast<double*>(com_buf.ptr);
     for (size_t i = 0; i < n; i++) {
         Eigen::Vector3d v;
         v(0) = com_ptr[i * 3 + 0];
         v(1) = com_ptr[i * 3 + 1];
         v(2) = com_ptr[i * 3 + 2];
         comPositions.push_back(v);
     }
 
     // Call the main computeJointTorques function with converted data.
     return computeJointTorques(q, q_dot, q_ddot, masses, inertiaMatrices, comPositions, DHparams);
 }


 py::tuple getMatricesWrapper(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& q_dot,
    py::array_t<double> masses_array,
    py::array_t<double> inertiaMatrices_array,
    py::array_t<double> comPositions_array,
    const Eigen::MatrixXd& DHparams)
{
    // Convert masses (expected 1D array)
    auto masses_buf = masses_array.request();
    if(masses_buf.ndim != 1)
        throw std::runtime_error("masses must be a 1D numpy array");
    size_t n = masses_buf.shape[0];
    std::vector<double> masses(n);
    double* masses_ptr = static_cast<double*>(masses_buf.ptr);
    for (size_t i = 0; i < n; i++) {
        masses[i] = masses_ptr[i];
    }

    // Convert inertiaMatrices (expected shape: (n, 3, 3))
    auto inertia_buf = inertiaMatrices_array.request();
    if(inertia_buf.ndim != 3)
        throw std::runtime_error("inertiaMatrices must be a 3D numpy array with shape (n,3,3)");
    if(inertia_buf.shape[1] != 3 || inertia_buf.shape[2] != 3)
        throw std::runtime_error("inertiaMatrices must have shape (n,3,3)");
    size_t n_inertia = inertia_buf.shape[0];
    if(n_inertia != n)
        throw std::runtime_error("Number of inertia matrices must equal number of masses");
    std::vector<Eigen::Matrix3d> inertiaMatrices;
    inertiaMatrices.reserve(n);
    double* inertia_ptr = static_cast<double*>(inertia_buf.ptr);
    for (size_t i = 0; i < n; i++) {
        Eigen::Matrix3d I;
        for (size_t r = 0; r < 3; r++) {
            for (size_t c = 0; c < 3; c++) {
                I(r, c) = inertia_ptr[i * 9 + r * 3 + c];
            }
        }
        inertiaMatrices.push_back(I);
    }

    // Convert comPositions (expected shape: (n, 3))
    auto com_buf = comPositions_array.request();
    if(com_buf.ndim != 2)
        throw std::runtime_error("comPositions must be a 2D numpy array with shape (n,3)");
    if(com_buf.shape[1] != 3)
        throw std::runtime_error("comPositions must have shape (n,3)");
    size_t n_com = com_buf.shape[0];
    if(n_com != n)
        throw std::runtime_error("Number of comPositions must equal number of masses");
    std::vector<Eigen::Vector3d> comPositions;
    comPositions.reserve(n);
    double* com_ptr = static_cast<double*>(com_buf.ptr);
    for (size_t i = 0; i < n; i++) {
        Eigen::Vector3d v;
        v(0) = com_ptr[i * 3 + 0];
        v(1) = com_ptr[i * 3 + 1];
        v(2) = com_ptr[i * 3 + 2];
        comPositions.push_back(v);
    }

    // Compute D, C, and g using our internal functions.
    Eigen::MatrixXd D = computeD(q, masses, inertiaMatrices, comPositions, DHparams);
    Eigen::MatrixXd C = computeC(q, q_dot, masses, inertiaMatrices, comPositions, DHparams);
    Eigen::VectorXd g = compute_g(q, masses, comPositions, DHparams);
    
    return py::make_tuple(D, C, g);
}




 
 // -----------------------------------------------------------------------------
 // pybind11 module definition
 PYBIND11_MODULE(compute_torque, m) {
    m.doc() = "Compute joint torques using DH-based dynamics with all inputs as numpy arrays.";
    m.def("computeJointTorques", &computeJointTorquesWrapper,
          py::arg("q"),
          py::arg("q_dot"),
          py::arg("q_ddot"),
          py::arg("masses"),
          py::arg("inertiaMatrices"),
          py::arg("comPositions"),
          py::arg("DHparams"),
          "Compute joint torques given robot state, parameters, and DH parameters.");
    
    // Expose the new getMatricesWrapper function to return D, C, and g.
    m.def("getMatrices", &getMatricesWrapper,
          py::arg("q"),
          py::arg("q_dot"),
          py::arg("masses"),
          py::arg("inertiaMatrices"),
          py::arg("comPositions"),
          py::arg("DHparams"),
          "Return the D, C, and g matrices for debugging purposes.");
}

 