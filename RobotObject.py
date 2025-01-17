import FreeCAD
import FreeCADGui


class RobotObject:
    def __init__(self, obj, links):
        # Link the custom object to this proxy
        obj.Proxy = self
        
        # Add the links attribute
        obj.addProperty("App::PropertyPythonObject", "Links", "Robot", "List of links").Links = links
        
        # Additional setup if needed
        self.Type = "Robot"

    def execute(self, obj):
        """Define how the object behaves when updated."""
        FreeCAD.Console.PrintMessage("Executing RobotObject\n")
    
    def onChanged(self, obj, prop):
        """Handle property changes."""
        if prop == "Links":
            FreeCAD.Console.PrintMessage(f"Links updated: {obj.Links}\n")

class RobotViewProvider:
    def __init__(self, obj):
        obj.Proxy = self

    def attach(self, obj):
        """Called when attached to an object."""
        self.Object = obj

    def getIcon(self):
        """Define a custom icon."""
        return ":/icons/Robot.svg"  # Change this to an actual icon path if you have one

    def updateData(self, fp, prop):
        """Called when the data of the object is updated."""
        pass

    def setEdit(self, vobj, mode):
        """Called when entering edit mode."""
        return False

    def unsetEdit(self, vobj, mode):
        """Called when leaving edit mode."""
        return False

    def doubleClicked(self, vobj):
        """Handle double-clicking on the object in the Tree View."""
        FreeCAD.Console.PrintMessage("Robot object was double-clicked\n")
        return True
    

def initialize_robot(links):
    """Directly initialize the robot object in the active document."""
    doc = FreeCAD.ActiveDocument
    if doc is None:
        doc = FreeCAD.newDocument()
    
    obj = doc.addObject("App::FeaturePython", "Robot")
    RobotObject(obj, links)  # Initialize the custom robot object
    RobotViewProvider(obj.ViewObject)  # Attach the view provider
    doc.recompute()
    return obj
