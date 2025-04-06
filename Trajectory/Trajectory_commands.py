import FreeCAD
import FreeCADGui

from PySide import QtCore
from main_utils import currentSelectionType, get_robot, displayMatrix, vec_to_numpy, mat_to_numpy
import os
import numpy as np
import cProfile
import pstats




class SolveTrajectoryCommand:
    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(os.path.dirname(__file__)), 'Resources', 'icons', 'gear.svg'),
            'MenuText': 'Solve Trajectory',
            'ToolTip': 'Solve inverse kinematics for chosen trajectory'
        }

    def Activated(self):
        profiler = cProfile.Profile()
        profiler.enable()

        sel = FreeCADGui.Selection.getSelection()
        if sel:
            trajectory_obj = sel[0]
            # Simply call the child's solve() method via the proxy:
            trajectory_obj.Proxy.solve()
            # Optionally call plotTrajectoryData():
            trajectory_obj.Proxy.plotTrajectoryData()

        profiler.disable()
        stats = pstats.Stats(profiler)
        stats.strip_dirs().sort_stats(pstats.SortKey.CUMULATIVE)
        sorted_stats = sorted(stats.stats.items(), key=lambda item: item[1][3], reverse=True)

        print("\nProfiling Results (Functions > 0.5 sec):")
        header_format = "{:<60s} {:>10s}"
        print(header_format.format("Function", "Time (sec)"))
        print("-" * 72)
        for func, stat in sorted_stats:
            cumulative_time = stat[3]
            if cumulative_time > 0.5:
                func_str = f"{func[2]} ({func[0]}:{func[1]})"
                print(header_format.format(func_str, f"{cumulative_time:.2f}"))

        print("\nProfiling completed.")

    def IsActive(self):
        sel = FreeCADGui.Selection.getSelection()
        if not sel:
            return False
        obj = sel[0]
        # Activate if the selected object has a `Type` property == 'Trajectory'
        return bool(obj and hasattr(obj, 'Type') and obj.Type == 'Trajectory')


FreeCADGui.addCommand('SolveTrajectoryCommand', SolveTrajectoryCommand())





# Utility variables for animation
animation_state = "stopped"  # Possible states: "playing", "paused", "stopped"
current_animation_index = 0
animation_timer = None

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
        if not sel.q:
            print("No angles calculated!")
            return

        if not animation_timer:
            animation_timer = QtCore.QTimer()
            animation_timer.timeout.connect(self.update_animation)

        if animation_state == "stopped":
            current_animation_index = 0
            self.update_robot_position(sel.q[0])

        animation_state = "playing"
        # Calculate delay based on the first interval
        if len(sel.t) > current_animation_index + 1:
            delay = int((sel.t[current_animation_index + 1] - sel.t[current_animation_index]) * 1000)
        else:
            delay = 100  # fallback delay if data is insufficient
        animation_timer.start(delay)

    def update_animation(self):
        global animation_state, current_animation_index, animation_timer
        sel = FreeCADGui.Selection.getSelection()[0]
        # Stop if we've reached the end of the trajectory data
        if not sel or current_animation_index >= len(sel.q) - 1:
            self.stop_animation()
            return

        self.update_robot_position(sel.q[current_animation_index])
        current_animation_index += 1

        # Update the delay dynamically for the next frame
        if current_animation_index < len(sel.t) - 1:
            new_delay = int((sel.t[current_animation_index + 1] - sel.t[current_animation_index]) * 1000)
            animation_timer.start(new_delay)
        else:
            self.stop_animation()

    def update_robot_position(self, q):
        robot = get_robot()
        if robot:
            robot.Angles = np.rad2deg(q).tolist()
            FreeCADGui.updateGui()

    def stop_animation(self):
        global animation_state, animation_timer
        if animation_timer:
            animation_timer.stop()
        animation_state = "stopped"

    def IsActive(self):
        sel = FreeCADGui.Selection.getSelection()[0] if FreeCADGui.Selection.getSelection() else None
        return bool(animation_state != "playing" and sel and len(sel.q) > 0)



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
        if sel.q:
            get_robot().Angles = np.rad2deg(sel.q[0]).tolist()
            FreeCADGui.updateGui()

        if animation_timer:
            animation_timer.stop()

    def IsActive(self):
        return animation_state in ["playing", "paused"]


FreeCADGui.addCommand('StopTrajectoryCommand', StopTrajectoryCommand())

