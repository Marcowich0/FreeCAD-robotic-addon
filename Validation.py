import FreeCAD
import FreeCADGui
import os
import numpy as np

from main_utils import get_robot, displayMatrix
import compute_torque

from roboticstoolbox import DHRobot, RevoluteDH

class printDynamics:
    """Command to draw the robot using Denavit-Hartenberg parameters."""

    def __init__(self):
        pass

    def GetResources(self):
        return {
            'Pixmap': os.path.join(os.path.dirname(__file__), 'Resources', 'icons', 'rotateBody.svg'),
            'MenuText': 'Rotate Joint',
            'ToolTip': 'Rotate the zero-state of a joint 90 degrees'
        }
    
    def Activated(self):
        print("Printing Dynamics")
        robot = get_robot()
        M = np.array(robot.Masses[1:])
        InertiaMatrices = np.array([np.array(m) for m in robot.InertiaMatrices[1:]])
        CenterOfMass = np.array([np.array(m) for m in robot.CenterOfMass[1:]])
        DHperameters = np.array(robot.DHPerameters)
        DHperameters[:,0] = 0
        DHperameters = DHperameters.astype(float)
        DHperameters[:,1:3] /= 1000

        q = np.deg2rad(robot.Angles)

        print("Joint torques C++")
        print("q_dot = 0, q_ddot = 0")
        displayMatrix(compute_torque.computeJointTorques(q, np.zeros_like(robot.Angles), np.zeros_like(robot.Angles), M, InertiaMatrices, CenterOfMass, DHperameters))
        print("q_dot = 1, q_ddot = 0")
        displayMatrix(compute_torque.computeJointTorques(q, np.ones_like(robot.Angles), np.zeros_like(robot.Angles), M, InertiaMatrices, CenterOfMass, DHperameters))
        print("q_dot = 0, q_ddot = 1")
        displayMatrix(compute_torque.computeJointTorques(q, np.zeros_like(robot.Angles), np.ones_like(robot.Angles), M, InertiaMatrices, CenterOfMass, DHperameters))




    def IsActive(self):        

        return True

FreeCADGui.addCommand('printDynamics', printDynamics())
