import FreeCAD
import FreeCADGui

import forward_kinematics
import RobotObject
import angle_sliders
import inverse_kinematics

import os

class Robotics (Workbench):
    MenuText = "Robotics"
    ToolTip = "Workbench for trajectory planning and control of robotic manipulators"
    #Icon = """paste here the contents of a 16x16 xpm icon"""

    def Initialize(self):
        self.__class__.Icon = os.path.join(FreeCAD.getHomePath(), 'Mod', 'FreeCAD-robotic-addon', 'Resources', 'icons', 'robotArm.svg')

        
        self.appendToolbar("Instantiation", ["CreateRobotCommand"])
        self.appendToolbar("Initial position", ["changeRotationDirectionCommand", "rotateBodyZeroCommand", "defineEndEffectorCommand"])
        self.appendToolbar("Forward Kinematics", ["FindDHParametersCommand", "RobotAnglesCommand"])
        self.appendToolbar("Inverse Kinematics", ["ToTargetPointCommand"])
        self.appendToolbar("Test", ["testCommand"])


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