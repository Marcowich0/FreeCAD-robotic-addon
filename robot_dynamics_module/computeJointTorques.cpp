/*
 * computeJointTorques.cpp
 *
 * Complete single-file implementation for computing robot joint torques 
 * using DH-based dynamics. The DH parameters are passed as a NumPy matrix 
 * (each row: [theta_offset, d, a, alpha]) and are used to compute the 
 * cumulative transforms.
 *
 * This file uses Eigen for matrix operations, Eigen::AutoDiff for automatic differentiation,
 * and pybind11 to expose the function to Python.
 */

 #include <pybind11/pybind11.h>
 #include <pybind11/eigen.h>
 #include <pybind11/numpy.h>
 #include <Eigen/Dense>
 #include <unsupported/Eigen/AutoDiff>
 #include <vector>
 #include <cmath>
 #include <functional>
 #include <stdexcept>
 
 namespace py = pybind11;
 
 // Define autodiff scalar type with appropriate number of derivatives
 using AutoDiffScalar = Eigen::AutoDiffScalar<Eigen::VectorXd>;
 
 // Create vector and matrix types using AutoDiffScalar
 template<int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
 using AutoDiffMatrix = Eigen::Matrix<AutoDiffScalar, Rows, Cols>;
 
 // Helper to convert regular Eigen::Vector to AutoDiff vector with derivatives initialized
 template<typename Derived>
 Eigen::Matrix<AutoDiffScalar, Eigen::Dynamic, 1> toAutoDiff(
     const Eigen::MatrixBase<Derived>& x, int wrt_idx = -1, int num_vars = -1) 
 {
     if (num_vars < 0) num_vars = x.size();
     Eigen::Matrix<AutoDiffScalar, Eigen::Dynamic, 1> result(x.size());
 
     for (int i = 0; i < x.size(); ++i) {
         if (wrt_idx < 0) {
             // Initialize with identity Jacobian (for all variables)
             Eigen::VectorXd derivatives = Eigen::VectorXd::Zero(num_vars);
             derivatives(i) = 1.0;
             result(i) = AutoDiffScalar(x(i), derivatives);
         } else if (i == wrt_idx) {
             // Initialize derivative only w.r.t. one variable
             Eigen::VectorXd derivatives = Eigen::VectorXd::Zero(1);
             derivatives(0) = 1.0;
             result(i) = AutoDiffScalar(x(i), derivatives);
         } else {
             // No derivatives
             result(i) = AutoDiffScalar(x(i), Eigen::VectorXd::Zero(wrt_idx >= 0 ? 1 : num_vars));
         }
     }
     return result;
 }
 
 // Extract value part from AutoDiff vector
 template<typename Derived>
 Eigen::VectorXd extractValue(const Eigen::MatrixBase<Derived>& x) {
     Eigen::VectorXd result(x.size());
     for (int i = 0; i < x.size(); ++i) {
         result(i) = x(i).value();
     }
     return result;
 }
 
 // Extract derivatives from AutoDiff vector
 template<typename Derived>
 Eigen::MatrixXd extractJacobian(const Eigen::MatrixBase<Derived>& x) {
     if (x.size() == 0) return Eigen::MatrixXd(0, 0);
     int n_vars = x(0).derivatives().size();
     Eigen::MatrixXd result(x.size(), n_vars);
     for (int i = 0; i < x.size(); ++i) {
         result.row(i) = x(i).derivatives();
     }
     return result;
 }
 
 // -----------------------------------------------------------------------------
 // getDHTransformations with AutoDiff support
 // Computes the cumulative transformation matrices given joint angles q and 
 // DH parameters (each row: [theta_offset, d, a, alpha]). 
 template<typename Scalar>
 std::vector<Eigen::Matrix<Scalar, 4, 4>> getDHTransformations(
     const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& q,
     const Eigen::MatrixXd& DHparams)
 {
     size_t n = static_cast<size_t>(q.size());
     std::vector<Eigen::Matrix<Scalar, 4, 4>> transforms;
     transforms.reserve(n);
     Eigen::Matrix<Scalar, 4, 4> T = Eigen::Matrix<Scalar, 4, 4>::Identity();
     
     for (size_t i = 0; i < n; ++i) {
         double theta_offset = DHparams(i, 0);   // offset for theta
         Scalar theta = q(i) + theta_offset;     // effective joint angle
         double d     = DHparams(i, 1);
         double a     = DHparams(i, 2);
         double alpha = DHparams(i, 3);
         
         Eigen::Matrix<Scalar, 4, 4> A;
         A <<  cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),  a*cos(theta),
               sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha),  a*sin(theta),
               0,           sin(alpha),             cos(alpha),             d,
               0,           0,                      0,                      1;
         T = T * A;
         transforms.push_back(T);
     }
     return transforms;
 }
 
 // -----------------------------------------------------------------------------
 // getJacobianCenter with AutoDiff support
 // For each link, compute the 6 x n Jacobian for the link's center of mass 
 // using the transforms and the given COM positions (in the link's local frame).
 template<typename Scalar>
 std::vector<Eigen::Matrix<Scalar, 6, Eigen::Dynamic>>
 getJacobianCenter(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& q,
                   const std::vector<Eigen::Matrix<Scalar, 4, 4>>& transforms,
                   const std::vector<Eigen::Vector3d>& comPositions)
 {
     size_t n = static_cast<size_t>(q.size());
     std::vector<Eigen::Matrix<Scalar, 6, Eigen::Dynamic>> jacobians;
     jacobians.reserve(n);
     
     const Eigen::Vector3d baseZ(0, 0, 1);
     const Eigen::Vector3d baseOrigin(0, 0, 0);
     
     for (size_t i = 0; i < n; ++i) {
         // Transform COM to global coordinates.
         Eigen::Matrix<Scalar, 4, 1> comLocalH;
         comLocalH << comPositions[i](0), comPositions[i](1), comPositions[i](2), Scalar(1.0);
         
         Eigen::Matrix<Scalar, 4, 1> comGlobalH = transforms[i] * comLocalH;
         Eigen::Matrix<Scalar, 3, 1> comGlobal = comGlobalH.template head<3>();
         
         // Initialize a 6xn Jacobian for link i.
         Eigen::Matrix<Scalar, 6, Eigen::Dynamic> J(6, n);
         J.setZero();
         
         for (size_t j = 0; j < n; ++j) {
             Eigen::Matrix<Scalar, 3, 1> zAxis, origin_j;
             if (j == 0) {
                 zAxis = baseZ.template cast<Scalar>();
                 origin_j = baseOrigin.template cast<Scalar>();
             } else {
                 zAxis = transforms[j - 1].template block<3,1>(0, 2);
                 origin_j = transforms[j - 1].template block<3,1>(0, 3);
             }
             
             if (j <= i) {
                 Eigen::Matrix<Scalar, 3, 1> r = comGlobal - origin_j;
                 Eigen::Matrix<Scalar, 3, 1> Jv = zAxis.cross(r);
                 Eigen::Matrix<Scalar, 3, 1> Jw = zAxis;
                 J.template block<3,1>(0, j) = Jv;
                 J.template block<3,1>(3, j) = Jw;
             }
         }
         jacobians.push_back(J);
     }
     return jacobians;
 }
 
 // -----------------------------------------------------------------------------
 // computeD with AutoDiff support
 // Computes the inertia matrix D(q) = Σ [ m_i * (Jv_i^T Jv_i) + (Jw_i^T I_global_i Jw_i) ]
 template<typename Scalar>
 Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> 
 computeD(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& q,
          const std::vector<double>& masses,
          const std::vector<Eigen::Matrix3d>& inertiaMatrices,
          const std::vector<Eigen::Vector3d>& comPositions,
          const Eigen::MatrixXd& DHparams)
 {
     size_t n = static_cast<size_t>(q.size());
     Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> D = 
         Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(n, n);
     
     std::vector<Eigen::Matrix<Scalar, 4, 4>> transforms = getDHTransformations(q, DHparams);
     std::vector<Eigen::Matrix<Scalar, 6, Eigen::Dynamic>> jacobians = 
         getJacobianCenter(q, transforms, comPositions);
     
     for (size_t i = 0; i < n; ++i) {
         Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Jv_i = jacobians[i].template topRows<3>();
         Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Jw_i = jacobians[i].template bottomRows<3>();
         Eigen::Matrix<Scalar, 3, 3> R_i = transforms[i].template block<3,3>(0, 0);
         
         // Convert inertia matrix to AutoDiff
         Eigen::Matrix<Scalar, 3, 3> I_i;
         for (int r = 0; r < 3; ++r) {
             for (int c = 0; c < 3; ++c) {
                 I_i(r, c) = Scalar(inertiaMatrices[i](r, c));
             }
         }
         
         Eigen::Matrix<Scalar, 3, 3> I_global = R_i * I_i * R_i.transpose();
         
         D += Scalar(masses[i]) * (Jv_i.transpose() * Jv_i)
            + (Jw_i.transpose() * I_global * Jw_i);
     }
     return D;
 }
 
 // -----------------------------------------------------------------------------
 // computeP with AutoDiff support
 // Computes potential energy P(q) = Σ m_i * g^T * com_global_i.
 template<typename Scalar>
 Scalar computeP(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& q,
                 const std::vector<double>& masses,
                 const std::vector<Eigen::Vector3d>& comPositions,
                 const Eigen::MatrixXd& DHparams)
 {
     size_t n = static_cast<size_t>(q.size());
     const Eigen::Vector3d gravity(0, 0, -9.81);
     Scalar P = Scalar(0.0);
     
     std::vector<Eigen::Matrix<Scalar, 4, 4>> transforms = getDHTransformations(q, DHparams);
     
     for (size_t i = 0; i < n; ++i) {
         Eigen::Matrix<Scalar, 4, 1> comLocalH;
         comLocalH << comPositions[i](0), comPositions[i](1), comPositions[i](2), Scalar(1.0);
         
         Eigen::Matrix<Scalar, 4, 1> comGlobalH = transforms[i] * comLocalH;
         Eigen::Matrix<Scalar, 3, 1> comGlobal = comGlobalH.template head<3>();
         
         // Convert gravity to AutoDiff scalar
         Eigen::Matrix<Scalar, 3, 1> g;
         g << Scalar(gravity(0)), Scalar(gravity(1)), Scalar(gravity(2));
         
         P += Scalar(masses[i]) * g.dot(comGlobal);
     }
     return P;
 }
 
 // -----------------------------------------------------------------------------
 // compute_g
 // Computes gravity torque vector g(q) = ∂P/∂q using automatic differentiation
 Eigen::VectorXd compute_g(const Eigen::VectorXd& q,
                           const std::vector<double>& masses,
                           const std::vector<Eigen::Vector3d>& comPositions,
                           const Eigen::MatrixXd& DHparams)
 {
     int n = q.size();
     Eigen::VectorXd g_vec(n);
     
     // Convert to AutoDiff with derivatives initialized
     auto q_ad = toAutoDiff(q);
     
     // Compute potential energy with AutoDiff
     AutoDiffScalar P = computeP(q_ad, masses, comPositions, DHparams);
     
     // Extract gradient from derivatives
     for (int i = 0; i < n; ++i) {
         g_vec(i) = P.derivatives()(i);
     }
     
     return g_vec;
 }
 
 // -----------------------------------------------------------------------------
 // computeC
 // Computes the Coriolis matrix C(q, q_dot) using automatic differentiation of D(q).
 Eigen::MatrixXd computeC(const Eigen::VectorXd& q,
                          const Eigen::VectorXd& q_dot,
                          const std::vector<double>& masses,
                          const std::vector<Eigen::Matrix3d>& inertiaMatrices,
                          const std::vector<Eigen::Vector3d>& comPositions,
                          const Eigen::MatrixXd& DHparams)
 {
     int n = q.size();
     Eigen::MatrixXd Cmat = Eigen::MatrixXd::Zero(n, n);
     
     // For each k, j pair, compute the Coriolis element C_kj
     for (int k = 0; k < n; ++k) {
         for (int j = 0; j < n; ++j) {
             double c_kj = 0.0;
             
             // For each i, compute the Christoffel symbols using automatic differentiation
             for (int i = 0; i < n; ++i) {
                 // Use AutoDiff to compute dD_dqi
                 auto q_ad = toAutoDiff(q);
                 auto D_ad = computeD(q_ad, masses, inertiaMatrices, comPositions, DHparams);
                 
                 // Extract the derivatives of D(k,j) w.r.t q(i)
                 double dD_dqi_kj = D_ad(k, j).derivatives()(i);
                 
                 // Extract the derivatives of D(k,i) w.r.t q(j)
                 double dD_dqj_ki = D_ad(k, i).derivatives()(j);
                 
                 // Extract the derivatives of D(i,j) w.r.t q(k)
                 double dD_dqk_ij = D_ad(i, j).derivatives()(k);
                 
                 // Compute the Christoffel symbol and multiply by q_dot(i)
                 c_kj += 0.5 * (dD_dqi_kj + dD_dqj_ki - dD_dqk_ij) * q_dot(i);
             }
             
             Cmat(k, j) = c_kj;
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
     m.doc() = "Compute joint torques using DH-based dynamics with automatic differentiation.";
     m.def("computeJointTorques", &computeJointTorquesWrapper,
           py::arg("q"),
           py::arg("q_dot"),
           py::arg("q_ddot"),
           py::arg("masses"),
           py::arg("inertiaMatrices"),
           py::arg("comPositions"),
           py::arg("DHparams"),
           "Compute joint torques given robot state, parameters, and DH parameters.");
     
     // Expose the getMatricesWrapper function to return D, C, and g.
     m.def("getMatrices", &getMatricesWrapper,
           py::arg("q"),
           py::arg("q_dot"),
           py::arg("masses"),
           py::arg("inertiaMatrices"),
           py::arg("comPositions"),
           py::arg("DHparams"),
           "Return the D, C, and g matrices for debugging purposes.");
 }