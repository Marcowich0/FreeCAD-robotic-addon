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




from PySide import QtGui
import csv
import math
import os
import numpy as np

class SaveTrajectoryDataCommand:
    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(os.path.dirname(__file__)), 'Resources', 'icons', 'save.svg'),
            'MenuText': 'Save Trajectory Data',
            'ToolTip': (
                'Save trajectory time, angles (converted to radians in [-π, π] with jump filtering), '
                'velocities, accelerations, torques, along with both the spline path and the original points. '
                'Data is downsampled to reduce file size.'
            )
        }

    def Activated(self):
        sel_list = FreeCADGui.Selection.getSelection()
        if not sel_list:
            print("No object selected!")
            return

        traj_obj = sel_list[0]
        if not hasattr(traj_obj, 'Type') or traj_obj.Type != 'Trajectory':
            print("Selected object is not a trajectory!")
            return

        try:
            t = list(traj_obj.t)
            angles = list(traj_obj.q)
            q_dot = list(traj_obj.q_dot)
            q_ddot = list(traj_obj.q_ddot)
            torques = list(traj_obj.Torques)
            orig_points = traj_obj.Points
            spline_points = getattr(traj_obj, 'SplinePoints', None)  # safer: might not exist
        except Exception as e:
            print("Error accessing trajectory data:", e)
            return

        # Convert FreeCAD.Vectors to lists (x, y, z)
        orig_points_list = []
        for pt in orig_points:
            if hasattr(pt, 'x'):
                orig_points_list.append([pt.x/1000, pt.y/1000, pt.z/1000])
            else:
                orig_points_list.append(pt)

        spline_points_list = []
        if spline_points:
            for pt in spline_points:
                if hasattr(pt, 'x'):
                    spline_points_list.append([pt.x, pt.y, pt.z])
                else:
                    spline_points_list.append(pt)

        n_rows = len(t)
        if not (len(angles) == len(q_dot) == len(q_ddot) == len(torques) == n_rows):
            print("Mismatch in main trajectory data lengths!")
            return

        max_t = max(t) if max(t) != 0 else 1

        def get_num_joints(data):
            if isinstance(data[0], (list, tuple, np.ndarray)):
                return len(data[0])
            else:
                return 1

        num_joints_angles = get_num_joints(angles)
        num_joints_q_dot = get_num_joints(q_dot)
        num_joints_q_ddot = get_num_joints(q_ddot)
        num_joints_torques = get_num_joints(torques)

        def convert_angle(angle_rad):
            return ((angle_rad + math.pi) % (2*math.pi)) - math.pi


        def angle_diff(a, b):
            return abs(a - b)

        threshold = 1.0
        downsample_factor = 5
        prev_angles = [None]*num_joints_angles

        header = ["time", "time_norm"]
        header += [f"angle_{i+1}" for i in range(num_joints_angles)]
        header += [f"q_dot_{i+1}" for i in range(num_joints_q_dot)]
        header += [f"q_ddot_{i+1}" for i in range(num_joints_q_ddot)]
        header += [f"torque_{i+1}" for i in range(num_joints_torques)]

        main_fileName, _ = QtGui.QFileDialog.getSaveFileName(None, "Save Trajectory Data", "", "CSV Files (*.csv)")
        if not main_fileName:
            print("No file selected; data not saved.")
            return

        try:
            with open(main_fileName, "w", newline="") as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(header)
                for i in range(0, n_rows, downsample_factor):
                    if float(t[i]) == 0.0:
                        continue
                    row = [t[i], t[i]/max_t]
                    # angles
                    if num_joints_angles > 1:
                        converted_angles = []
                        for j, a in enumerate(angles[i]):
                            new_angle = convert_angle(a)
                            if prev_angles[j] is not None:
                                if angle_diff(new_angle, prev_angles[j]) > threshold:
                                    converted_angles.append(float('nan'))
                                else:
                                    converted_angles.append(new_angle)
                                prev_angles[j] = new_angle
                            else:
                                prev_angles[j] = new_angle
                                converted_angles.append(new_angle)
                        row.extend(converted_angles)
                    else:
                        new_angle = convert_angle(angles[i])
                        if prev_angles[0] is not None:
                            if angle_diff(new_angle, prev_angles[0]) > threshold:
                                row.append(float('nan'))
                            else:
                                row.append(new_angle)
                            prev_angles[0] = new_angle
                        else:
                            prev_angles[0] = new_angle
                            row.append(new_angle)
                    # velocities
                    if num_joints_q_dot > 1:
                        row.extend(q_dot[i])
                    else:
                        row.append(q_dot[i])
                    # accelerations
                    if num_joints_q_ddot > 1:
                        row.extend(q_ddot[i])
                    else:
                        row.append(q_ddot[i])
                    # torques
                    if num_joints_torques > 1:
                        row.extend(torques[i])
                    else:
                        row.append(torques[i])
                    writer.writerow(row)
            print("Main trajectory data successfully saved as CSV to:", main_fileName)
        except Exception as e:
            print("Failed to save main trajectory data as CSV:", e)
            return

        # Save original points
        base, ext = os.path.splitext(main_fileName)
        orig_fileName = base + "_orig_points" + ext
        try:
            with open(orig_fileName, "w", newline="") as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(["point_x", "point_y", "point_z"])
                for pt in orig_points_list:
                    if len(pt) == 3:
                        writer.writerow(pt)
                    else:
                        writer.writerow([None, None, None])
            print("Original trajectory points successfully saved as CSV to:", orig_fileName)
        except Exception as e:
            print("Failed to save original trajectory points as CSV:", e)

        # Save spline points ONLY IF they exist
        if spline_points_list:
            spline_fileName = base + "_spline_points" + ext
            try:
                with open(spline_fileName, "w", newline="") as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(["point_x", "point_y", "point_z"])
                    for pt in spline_points_list:
                        if len(pt) == 3:
                            writer.writerow(pt)
                        else:
                            writer.writerow([None, None, None])
                print("Spline trajectory points successfully saved as CSV to:", spline_fileName)
            except Exception as e:
                print("Failed to save spline trajectory points as CSV:", e)

    def IsActive(self):
        sel_list = FreeCADGui.Selection.getSelection()
        if not sel_list:
            return False
        sel = sel_list[0]
        return hasattr(sel, 'Type') and sel.Type == 'Trajectory'


FreeCADGui.addCommand('SaveTrajectoryDataCommand', SaveTrajectoryDataCommand())