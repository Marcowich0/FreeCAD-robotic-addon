import FreeCAD
import FreeCADGui

class MyTestCommand:
    def GetResources(self):
        return {'Pixmap': '', 'MenuText': 'My Test Command', 'ToolTip': 'This is my test command'}

    def Activated(self):
        print("My Test Command ran successfully!")

    def IsActive(self):
        return True

FreeCADGui.addCommand('my_test_command', MyTestCommand())