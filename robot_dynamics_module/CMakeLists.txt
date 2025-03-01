cmake_minimum_required(VERSION 3.10)
project(compute_torque LANGUAGES CXX)

# Find Python and NumPy
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

# Add submodules for pybind11, Eigen, and autodiff
add_subdirectory(external/pybind11)
add_subdirectory(external/eigen)

# autodiff is header-only; create an interface library for it.
add_library(autodiff INTERFACE)
target_include_directories(autodiff INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/external/autodiff/include)

# Create the shared library
add_library(compute_torque MODULE computeJointTorques.cpp)

# Include directories: Python, NumPy and submodules (pybind11, Eigen, autodiff)
target_include_directories(compute_torque PRIVATE 
    ${Python3_INCLUDE_DIRS}
    "D:/FreeCAD/bin/Lib/site-packages/numpy/_core/include"
    ${CMAKE_CURRENT_SOURCE_DIR}/external/eigen
    ${CMAKE_CURRENT_SOURCE_DIR}/external/autodiff/include
)

# Link against Python, pybind11, and autodiff. (Eigen and autodiff are header-only.)
target_link_libraries(compute_torque PRIVATE ${Python3_LIBRARIES} pybind11::module autodiff)

# Ensure the output is a .pyd file (Python extension on Windows)
set_target_properties(compute_torque PROPERTIES PREFIX "" SUFFIX ".pyd")
