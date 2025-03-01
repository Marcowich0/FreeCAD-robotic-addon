cmake_minimum_required(VERSION 3.10)
project(compute_torque LANGUAGES CXX)

# Find Python and NumPy
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

# Add submodules for pybind11 and Eigen
add_subdirectory(external/pybind11)
add_subdirectory(external/eigen)

# Create the compute_torque module (dynamics)
add_library(compute_torque MODULE computeJointTorques.cpp)
target_include_directories(compute_torque PRIVATE 
    ${Python3_INCLUDE_DIRS}
    "D:/FreeCAD/bin/Lib/site-packages/numpy/_core/include"
    ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen
)
target_compile_features(compute_torque PRIVATE cxx_std_14)
target_link_libraries(compute_torque PRIVATE ${Python3_LIBRARIES} pybind11::module)
set_target_properties(compute_torque PROPERTIES PREFIX "" SUFFIX ".pyd")

# Create the inverse_kinematics module (IK)
add_library(inverse_kinematics_cpp MODULE inverseKinematicSolver.cpp)
target_include_directories(inverse_kinematics_cpp PRIVATE 
    ${Python3_INCLUDE_DIRS}
    "D:/FreeCAD/bin/Lib/site-packages/numpy/_core/include"
    ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen
)
target_compile_features(inverse_kinematics_cpp PRIVATE cxx_std_14)
target_link_libraries(inverse_kinematics_cpp PRIVATE ${Python3_LIBRARIES} pybind11::module)
set_target_properties(inverse_kinematics_cpp PROPERTIES PREFIX "" SUFFIX ".pyd")
