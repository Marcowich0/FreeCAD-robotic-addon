import FreeCAD
import FreeCADGui

from PySide import QtCore
from main_utils import currentSelectionType, get_robot, displayMatrix, vec_to_numpy, mat_to_numpy
import os

class PlayTrajectoryCommand:
    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(os.path.dirname(__file__)), 'Resources', 'icons', 'play.svg'),
            'MenuText': 'Play Trajectory',
            'ToolTip': 'Start/resume trajectory animation'
        }

    def Activated(self):
        global animation_state, current_animation_index, animation_timer
        if animation_state == "playing":
            return

        sel = FreeCADGui.Selection.getSelection()[0]
        if not sel.Angles:
            print("No angles calculated!")
            return

        if not animation_timer:
            animation_timer = QtCore.QTimer()
            animation_timer.timeout.connect(self.update_animation)

        if animation_state == "stopped":
            current_animation_index = 0
            self.update_robot_position(sel.Angles[0])

        animation_state = "playing"
        delay = int((sel.DistanceBetweenPoints / sel.Velocity) * 1000)
        animation_timer.start(delay)

    def update_animation(self):
        global animation_state, current_animation_index
        sel = FreeCADGui.Selection.getSelection()[0]
        if not sel or current_animation_index >= len(sel.Angles):
            self.stop_animation()
            return

        self.update_robot_position(sel.Angles[current_animation_index])
        current_animation_index += 1

    def update_robot_position(self, angles):
        robot = get_robot()
        if robot:
            robot.Angles = angles
            FreeCADGui.updateGui()

    def stop_animation(self):
        global animation_state, animation_timer
        if animation_timer:
            animation_timer.stop()
        animation_state = "stopped"

    def IsActive(self):
        sel = FreeCADGui.Selection.getSelection()[0] if FreeCADGui.Selection.getSelection() else None
        return bool(sel and hasattr(sel, 'Type') and sel.Type == 'Trajectory'
                    and animation_state != "playing" and sel.Angles)


FreeCADGui.addCommand('PlayTrajectoryCommand', PlayTrajectoryCommand())


class PauseTrajectoryCommand:
    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(os.path.dirname(__file__)), 'Resources', 'icons', 'pause.svg'),
            'MenuText': 'Pause Trajectory',
            'ToolTip': 'Pause trajectory animation'
        }

    def Activated(self):
        global animation_state, animation_timer
        if animation_state == "playing":
            animation_state = "paused"
            if animation_timer:
                animation_timer.stop()

    def IsActive(self):
        return animation_state == "playing"


FreeCADGui.addCommand('PauseTrajectoryCommand', PauseTrajectoryCommand())


class StopTrajectoryCommand:
    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(os.path.dirname(__file__)), 'Resources', 'icons', 'stop.svg'),
            'MenuText': 'Stop Trajectory',
            'ToolTip': 'Stop and reset trajectory animation'
        }

    def Activated(self):
        global animation_state, current_animation_index
        animation_state = "stopped"
        current_animation_index = 0

        sel = FreeCADGui.Selection.getSelection()[0]
        if sel.Angles:
            get_robot().Angles = sel.Angles[0]
            FreeCADGui.updateGui()

        if animation_timer:
            animation_timer.stop()

    def IsActive(self):
        return animation_state in ["playing", "paused"]


FreeCADGui.addCommand('StopTrajectoryCommand', StopTrajectoryCommand())

