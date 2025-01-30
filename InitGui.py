import FreeCAD
import FreeCADGui

import forward_kinematics
import RobotObject
import angle_sliders

import os

class Robotics (Workbench):
    MenuText = "Robotics"
    ToolTip = "Workbench for trajectory planning and control of robotic manipulators"
    #Icon = """paste here the contents of a 16x16 xpm icon"""

    def Initialize(self):
        self.__class__.Icon = os.path.join(FreeCAD.getHomePath(), 'Mod', 'FreeCAD-robotic-addon', 'Resources', 'icons', 'robotArm.svg')

        self.list = [ "CreateRobotCommand", "changeRotationDirectionCommand", "rotateBodyZeroCommand", "FindDHParametersCommand", "RobotAnglesCommand", "defineEndEffectorCommand" , "testCommand" ]
        self.appendToolbar("My Commands", self.list)
        self.appendMenu("My New Menu", self.list)
        self.appendMenu(["An existing Menu", "My submenu"], self.list)

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