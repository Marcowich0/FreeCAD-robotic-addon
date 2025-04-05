import FreeCAD
import FreeCADGui

import inspect
import sys  
import os

import qt_angle_sliders
import qt_orientation_sliders
import qt_move_end_effector

import RobotObject     
import Trajectory_parent
import Trajectory_constant_velocity
import Trajectory_commands

import forward_kinematics
import inverse_kinematics

import Validation
import TimeOptimizedTrajectory

import dynamics

class Robotics (Workbench):
    MenuText = "Robotics"
    ToolTip = "Workbench for trajectory planning and control of robotic manipulators"
    #Icon = """paste here the contents of a 16x16 xpm icon"""

    def Initialize(self):
        for path in sys.path:
            print(path)
        
        self.__class__.Icon = os.path.join(FreeCAD.getHomePath(), 'Mod', 'FreeCAD-robotic-addon', 'Resources', 'icons', 'robotArm.svg')

        
        self.appendToolbar("Instantiation", ["CreateRobotCommand", "RemoveRobotCommand"])
        self.appendToolbar("Initial position", ["rotateBodyZeroCommand", "defineEndEffectorCommand", "OrientationCommand"])
        self.appendToolbar("Forward Kinematics", [ "RobotAnglesCommand"])
        self.appendToolbar("Inverse Kinematics", [ "MoveEndEffectorCommand","ToTargetPointCommand"])
        self.appendToolbar("Trajectory", ["AddConstantVelTrajectoryCommand", "SolveTrajectoryCommand", "PlayTrajectoryCommand", "PauseTrajectoryCommand", "StopTrajectoryCommand", "SaveTrajectoryDataCommand"])
        self.appendToolbar("Validation", ["printDynamics"])
        self.appendToolbar("Test", ["computeTimeOptimizedTrajectory"])

        #self.appendMenu("My New Menu", self.list)
        #self.appendMenu(["An existing Menu", "My submenu"], self.list)

    def Activated(self):
        print(self.__class__.Icon)
        return

    def Deactivated(self):
        return

    def ContextMenu(self, recipient):
        self.appendContextMenu("My commands", self.list)

    def GetClassName(self):
        return "Gui::PythonWorkbench"

FreeCADGui.addWorkbench(Robotics())