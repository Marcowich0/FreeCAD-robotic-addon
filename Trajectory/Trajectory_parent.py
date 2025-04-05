import FreeCAD
import numpy as np
import matplotlib.pyplot as plt
from Utils.main_utils import get_robot
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
        DHperameters = np.array(robot.DHPerameters)
        DHperameters[:, 0] = 0
        DHperameters = DHperameters.astype(float)
        DHperameters[:, 1:3] /= 1000

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
