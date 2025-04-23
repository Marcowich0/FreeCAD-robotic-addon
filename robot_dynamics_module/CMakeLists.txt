set(MY_WORKBENCH_VERSION_MAJOR 1)
set(MY_WORKBENCH_VERSION_MINOR 0)
set(MY_WORKBENCH_VERSION_PATCH 0)
set(MY_WORKBENCH_VERSION "${MY_WORKBENCH_VERSION_MAJOR}.${MY_WORKBENCH_VERSION_MINOR}.${MY_WORKBENCH_VERSION_PATCH}")

cmake_minimum_required(VERSION 3.15)
project(freecad_robotics LANGUAGES CXX)

# Find Python and NumPy
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

# Add submodules for pybind11 and Eigen
add_subdirectory(external/pybind11)
add_subdirectory(external/eigen)

# compute_torque module
add_library(compute_torque MODULE computeJointTorques.cpp)
target_include_directories(compute_torque PRIVATE
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen
)
target_compile_features(compute_torque PRIVATE cxx_std_14)
target_link_libraries(compute_torque PRIVATE ${Python3_LIBRARIES} pybind11::module)
target_compile_definitions(compute_torque PRIVATE WORKBENCH_VERSION="${MY_WORKBENCH_VERSION}")
set_target_properties(compute_torque PROPERTIES PREFIX "" SUFFIX ".pyd")
install(TARGETS compute_torque
        LIBRARY DESTINATION freecad_robotics)

# inverse_kinematics_cpp module
add_library(inverse_kinematics_cpp MODULE inverseKinematicSolver.cpp)
target_include_directories(inverse_kinematics_cpp PRIVATE
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen
)
target_compile_features(inverse_kinematics_cpp PRIVATE cxx_std_14)
target_link_libraries(inverse_kinematics_cpp PRIVATE ${Python3_LIBRARIES} pybind11::module)
target_compile_definitions(inverse_kinematics_cpp PRIVATE WORKBENCH_VERSION="${MY_WORKBENCH_VERSION}")
set_target_properties(inverse_kinematics_cpp PROPERTIES PREFIX "" SUFFIX ".pyd")
install(TARGETS inverse_kinematics_cpp
        LIBRARY DESTINATION freecad_robotics)