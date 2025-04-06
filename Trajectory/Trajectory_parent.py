import FreeCAD
import FreeCADGui
import Part
import numpy as np
import matplotlib.pyplot as plt
from Utils.main_utils import *
import compute_torque


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
        obj.addProperty("App::PropertyVectorList", "Points", "Trajectory", "List of points").Points = []

        # Arrays that store solutions/time-series data.
        obj.addProperty("App::PropertyPythonObject", "t", "Time", "Times at each point").t = []
        obj.addProperty("App::PropertyPythonObject", "q", "Trajectory", "List of angles").q = []
        obj.addProperty("App::PropertyPythonObject", "q_dot", "Trajectory", "List of joint velocities").q_dot = []
        obj.addProperty("App::PropertyPythonObject", "q_ddot", "Trajectory", "List of joint accelerations").q_ddot = []
        obj.addProperty("App::PropertyPythonObject", "Torques", "Trajectory", "List of joint torques").Torques = []

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

    def updateTorques(self):
        robot = get_robot()  # or however you retrieve your robot
        M = np.array(robot.Masses[1:])
        InertiaMatrices = np.array([np.array(m) for m in robot.InertiaMatrices[1:]])
        CenterOfMass = np.array([np.array(m) for m in robot.CenterOfMass[1:]])

        # Prepare DH parameters
        DHperameters = getNumericalDH()

        # Fetch from self.Object
        q      = self.Object.q
        q_dot  = self.Object.q_dot
        q_ddot = self.Object.q_ddot

        # Compute torques for each time step
        tau = [
            compute_torque.computeJointTorques(
                qq, qd, qdd,
                M, InertiaMatrices, CenterOfMass, DHperameters
            )
            for qq, qd, qdd in zip(q, q_dot, q_ddot)
        ]

        # Store results back on the FreeCAD object
        self.Object.Torques = np.array(tau, dtype=float)

        FreeCAD.Console.PrintMessage("Torques updated successfully.\n")

    def plotTrajectoryData(self):
        """
        Plots the Trajectory's time-series data: positions, velocities, accelerations, and torques.
        """
        import matplotlib.pyplot as plt

        # For convenience, convert to numpy arrays
        t = np.array(self.Object.t)
        q = np.array(self.Object.q)
        q_dot = np.array(self.Object.q_dot)
        q_ddot = np.array(self.Object.q_ddot)
        torques = np.array(self.Object.Torques)

        # Dark background styling
        plt.style.use('dark_background')
        dark_gray = '#303030'
        plt.rcParams['axes.facecolor'] = dark_gray
        plt.rcParams['figure.facecolor'] = dark_gray

        # Helper function for discontinuities in wrapped angles
        def insert_nan_breaks(x, y, threshold=np.pi):
            new_x = [x[0]]
            new_y = [y[0]]
            for i in range(1, len(x)):
                if np.abs(y[i] - y[i - 1]) > threshold:
                    new_x.append(np.nan)
                    new_y.append(np.nan)
                new_x.append(x[i])
                new_y.append(y[i])
            return np.array(new_x), np.array(new_y)

        # Reshape if needed
        if q.ndim == 1:
            q = q.reshape(-1, 1)
            q_dot = q_dot.reshape(-1, 1)
            q_ddot = q_ddot.reshape(-1, 1)

        n_joints = q.shape[1]

        fig, axs = plt.subplots(4, 1, sharex=True, figsize=(8, 10))

        # Positions
        for j in range(n_joints):
            wrapped = (q[:, j] + np.pi) % (2 * np.pi) - np.pi
            new_t, new_wrapped = insert_nan_breaks(t, wrapped, np.pi)
            axs[0].plot(new_t, new_wrapped, label=f'Joint {j+1}')
        axs[0].set_ylabel('Position (rad)')
        axs[0].set_title('Joint Positions')
        axs[0].grid(True, color='gray')
        axs[0].set_ylim(-np.pi, np.pi)
        ticks = [-np.pi, -np.pi/2, 0, np.pi/2, np.pi]
        labels = [r'$-\pi$', r'$-\pi/2$', '0', r'$\pi/2$', r'$\pi$']
        axs[0].set_yticks(ticks)
        axs[0].set_yticklabels(labels)
        axs[0].legend(loc='upper right')

        # Velocities
        for j in range(n_joints):
            axs[1].plot(t, q_dot[:, j], label=f'Joint {j+1}')
        axs[1].set_ylabel('Velocity (rad/s)')
        axs[1].set_title('Joint Velocities')
        axs[1].grid(True, color='gray')
        axs[1].legend(loc='upper right')

        # Accelerations
        for j in range(n_joints):
            axs[2].plot(t, q_ddot[:, j], label=f'Joint {j+1}')
        axs[2].set_ylabel('Acceleration (rad/s²)')
        axs[2].set_title('Joint Accelerations')
        axs[2].grid(True, color='gray')
        axs[2].legend(loc='upper right')

        # Torques
        if torques.ndim == 1:
            torques = torques.reshape(-1, 1)
        for j in range(torques.shape[1]):
            axs[3].plot(t, torques[:, j], label=f'Joint {j+1}')
        axs[3].set_ylabel('Torque (Nm)')
        axs[3].set_xlabel('Time (s)')
        axs[3].set_title('Joint Torques')
        axs[3].grid(True, color='gray')
        axs[3].legend(loc='upper right')

        plt.tight_layout()
        plt.show()


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





#
# --------------------- HELPER FUNCTIONS ---------------------
#
def selectEdgesForTrajectory(trajectory_obj):
    collected_edges = []
    for sel in FreeCADGui.Selection.getSelectionEx():
        for subel_name in sel.SubElementNames:
            collected_edges.append((sel.Object, subel_name))

    trajectory_obj.Edges = collected_edges

    # Attempt to set 'Body' by searching parents
    if collected_edges:
        obj = collected_edges[0][0]
        for parent in obj.Parents:
            if parent[0].Name == 'Assembly':
                body_name = parent[1].split('.')[0]
                print(f"Found parent body: {body_name}")
                trajectory_obj.Body = FreeCAD.ActiveDocument.getObject(body_name)
                break

    print(f"Collected {len(collected_edges)} sub-elements in total.")





def computeTrajectoryPoints(trajectory_obj):
    # We assume there's at least one selected Trajectory in the GUI
    sel = FreeCADGui.Selection.getSelection()[0]
    if not trajectory_obj.Edges:
        print("Error: No edge stored in the trajectory object!")
        return []

    point_list = []
    for (obj, sub_elems) in trajectory_obj.Edges:
        # sub_elems might be a single name or a list
        names = sub_elems if isinstance(sub_elems, (list, tuple)) else [sub_elems]
        for sub in names:
            print("edge array", obj, sub)
            edge_candidate = obj.getSubObject(sub)

            if isinstance(edge_candidate, tuple):
                edge = edge_candidate[0]
            else:
                edge = edge_candidate

            body = trajectory_obj.Body
            o_A_ol = mat_to_numpy(body.Placement.Matrix)
            print("Matrix used for transformation")
            displayMatrix(o_A_ol)

            # Determine number of points
            n_points = round(edge.Length * 1e-3 / trajectory_obj.DistanceBetweenPoints)
            if n_points < 2:
                n_points = 2

            if isinstance(edge.Curve, (Part.Line, Part.LineSegment)):
                start = edge.Vertexes[0].Point
                end = edge.Vertexes[-1].Point
                points_local = [start + (end - start)*(i/(n_points-1)) for i in range(n_points)]
            else:
                points_local = edge.discretize(Number=n_points)

            if sel.externalModel:
                # If externalModel, no transform?
                points_global = [(np.append(vec_to_numpy(pt), 1))[:3] for pt in points_local]
            else:
                # Apply body placement transform
                points_global = [(o_A_ol @ np.append(vec_to_numpy(pt), 1))[:3] for pt in points_local]

            print(f"Points on edge: {len(points_global)}")
            point_list.append(points_global)

    # Merge all lists of points
    merged_pts = chain_lists(point_list)
    trajectory_obj.Points = merged_pts
    return merged_pts




def chain_lists(list_of_lists):
    arrs = list_of_lists
    chain = arrs.pop(0)  # take the first
    while arrs:
        best_dist = float('inf')
        best_idx = -1
        best_flip = False
        best_prepend = False
        for i, seg in enumerate(arrs):
            cs, ce = chain[0], chain[-1]  # current chain start/end
            ss, se = seg[0], seg[-1]      # segment start/end
            # check 4 combos
            d1 = np.linalg.norm(ce - ss)  # append normal
            if d1 < best_dist:
                best_dist, best_idx, best_flip, best_prepend = d1, i, False, False
            d2 = np.linalg.norm(ce - se)  # append reversed
            if d2 < best_dist:
                best_dist, best_idx, best_flip, best_prepend = d2, i, True, False
            d3 = np.linalg.norm(cs - ss)  # prepend reversed
            if d3 < best_dist:
                best_dist, best_idx, best_flip, best_prepend = d3, i, True, True
            d4 = np.linalg.norm(cs - se)  # prepend normal
            if d4 < best_dist:
                best_dist, best_idx, best_flip, best_prepend = d4, i, False, True
        chosen = arrs.pop(best_idx)
        if best_flip:
            chosen.reverse()
        if best_prepend:
            chain = chosen[:-1] + chain
        else:
            chain = chain + chosen[1:]
    return [FreeCAD.Vector(*p) for p in chain]