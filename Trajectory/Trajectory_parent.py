import FreeCAD

class Trajectory:
    """
    Base trajectory class holding all FreeCAD properties.
    Child classes should override the solve() method.
    """

    def __init__(self, obj):
        obj.Proxy = self
        self.Object = obj

        # Basic trajectory properties.
        obj.addProperty("App::PropertyString", "Type", "Trajectory", "Type of the object").Type = "Trajectory"
        obj.addProperty("App::PropertyLink", "Body", "Trajectory", "Link to a Body")
        obj.addProperty("App::PropertyLinkSubList", "Edges", "Trajectory", "Link to Edges").Edges = []
        obj.addProperty("App::PropertyVectorList", "Points", "Trajectory", "List of points").Points = []
        obj.addProperty("App::PropertyVectorList", "SplinePoints", "Trajectory", "List of spline points").SplinePoints = []
        obj.addProperty("App::PropertyFloat", "Velocity", "Trajectory", "Velocity").Velocity = 1

        # Arrays that store solutions/time-series data.
        obj.addProperty("App::PropertyPythonObject", "t", "Time", "Times at each point").t = []
        obj.addProperty("App::PropertyPythonObject", "Angles", "Trajectory", "List of angles").Angles = []
        obj.addProperty("App::PropertyPythonObject", "q_dot", "Trajectory", "List of joint velocities").q_dot = []
        obj.addProperty("App::PropertyPythonObject", "q_ddot", "Trajectory", "List of joint accelerations").q_ddot = []
        obj.addProperty("App::PropertyPythonObject", "Torques", "Trajectory", "List of joint torques").Torques = []

        # Spline/tuning parameters
        obj.addProperty("App::PropertyFloat", "smoothing", "Trajectory", "Smoothing factor").smoothing = 1
        obj.addProperty("App::PropertyFloat", "alpha", "Trajectory", "Pre-path extension").alpha = 0.1

        obj.addProperty("App::PropertyBool", "externalModel", "Trajectory", "External Model").externalModel = False
        obj.addProperty("App::PropertyFloat", "DistanceBetweenPoints", "Trajectory", "Distance between points").DistanceBetweenPoints = 5e-3

    def solve(self):
        """
        Stub method. Child trajectory classes should implement their own solve() method.
        """
        FreeCAD.Console.PrintMessage("Trajectory parent: no solve() method implemented.\n")

    def execute(self, obj):
        """
        If needed, FreeCAD calls this on document recompute. 
        Typically used if the object is parametric. 
        """
        pass


class ViewProviderTrajectory:
    def __init__(self, vobj):
        vobj.Proxy = self

    def attach(self, vobj):
        self.Object = vobj.Object
        return

    def getIcon(self):
        return ":/icons/Trajectory.svg"

    def updateData(self, fp, prop):
        return

    def getDisplayModes(self, obj):
        return []

    def getDefaultDisplayMode(self):
        return "Shaded"

    def setDisplayMode(self, mode):
        return mode

    def onChanged(self, vp, prop):
        return

    def __getstate__(self):
        return {}

    def __setstate__(self, state):
        return None
