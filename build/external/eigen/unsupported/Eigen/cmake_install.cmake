# Install script for directory: D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files/freecad_robotics")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE FILE FILES
    "D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen/AdolcForward"
    "D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen/AlignedVector3"
    "D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen/ArpackSupport"
    "D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen/AutoDiff"
    "D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen/BVH"
    "D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen/EulerAngles"
    "D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen/FFT"
    "D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen/IterativeSolvers"
    "D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen/KroneckerProduct"
    "D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen/LevenbergMarquardt"
    "D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen/MatrixFunctions"
    "D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen/MPRealSupport"
    "D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen/NNLS"
    "D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen/NonLinearOptimization"
    "D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen/NumericalDiff"
    "D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen/OpenGLSupport"
    "D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen/Polynomials"
    "D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen/SparseExtra"
    "D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen/SpecialFunctions"
    "D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen/Splines"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE DIRECTORY FILES "D:/FreeCAD/Mod/FreeCAD-robotic-addon/robot_dynamics_module/external/eigen/unsupported/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("D:/FreeCAD/Mod/FreeCAD-robotic-addon/build/external/eigen/unsupported/Eigen/CXX11/cmake_install.cmake")

endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "D:/FreeCAD/Mod/FreeCAD-robotic-addon/build/external/eigen/unsupported/Eigen/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
