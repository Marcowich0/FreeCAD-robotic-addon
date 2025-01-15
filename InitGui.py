import FreeCAD
import FreeCADGui
import test_command

class Robotics (Workbench):
    MenuText = "Robotics"
    ToolTip = "Workbench for trajectory planning and control of robotic manipulators"
    Icon = """paste here the contents of a 16x16 xpm icon"""

    def Initialize(self):
        import test_command
        self.list = ["my_test_command"]
        self.appendToolbar("My Commands", self.list)
        self.appendMenu("My New Menu", self.list)
        self.appendMenu(["An existing Menu", "My submenu"], self.list)

    def Activated(self):
        return

    def Deactivated(self):
        return

    def ContextMenu(self, recipient):
        self.appendContextMenu("My commands", self.list)

    def GetClassName(self):
        return "Gui::PythonWorkbench"

Gui.addWorkbench(Robotics())