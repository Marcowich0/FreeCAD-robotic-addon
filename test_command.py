import FreeCAD
import FreeCADGui
import os

class MyTestCommand:
    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'robotArm.svg'),
            'MenuText': 'My Test Command 2',
            'ToolTip': 'This is my test command'
        }

    def Activated(self):
        print("My Test Command ran successfully!")
        FreeCAD.Console.PrintMessage("My Test Command ran successfully with long print")
        FreeCADGui.activeDocument().activeView().viewIsometric()
        FreeCADGui.runCommand('Std_ViewGroup',0)

    def IsActive(self):
        return True

FreeCADGui.addCommand('my_test_command', MyTestCommand())
