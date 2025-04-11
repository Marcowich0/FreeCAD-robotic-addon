import FreeCAD
import FreeCADGui
import os
import numpy as np

from Utils.main_utils import *
import compute_torque
import roboticstoolbox as rtb

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
        DHperameters = getNumericalDH()

        q = np.deg2rad(robot.Angles)

        print("Example results using:")
        print("Masses: ", M)
        print("Inertia Matrices: ")
        for m in InertiaMatrices:
             displayMatrix(m)
        print("Center of Mass: ", CenterOfMass)
        print("DH Parameters: ", DHperameters)
        print("Joint angles: ", q)

        # Example results using custom function

        print("Joint torques C++")
        print("q_dot = 0, q_ddot = 0")
        displayMatrix(compute_torque.computeJointTorques(q, np.zeros_like(robot.Angles), np.zeros_like(robot.Angles), M, InertiaMatrices, CenterOfMass, DHperameters))
        print("q_dot = 10, q_ddot = 0")
        displayMatrix(compute_torque.computeJointTorques(q, np.ones_like(robot.Angles)*10, np.zeros_like(robot.Angles), M, InertiaMatrices, CenterOfMass, DHperameters))
        print("q_dot = 0, q_ddot = 10")
        displayMatrix(compute_torque.computeJointTorques(q, np.zeros_like(robot.Angles), np.ones_like(robot.Angles)*10, M, InertiaMatrices, CenterOfMass, DHperameters))

        #Define robot using roboticstoolbox
        links = []
        num_joints = len(robot.DHPerameters)
        DH_rtb = np.copy(DHperameters)
        for i in range(num_joints):
                # Create DHLink object for each joint
                # Assuming Revolute joints ('R'). Change to 'P' for prismatic.
                # Using DH parameters with lengths in meters (DH_rtb)
                # Inertia tensor I should be 3x3 matrix
                # Center of mass r should be a 3-vector
                link = rtb.DHLink(
                    theta=DH_rtb[i, 0],     # DH theta offset (rad)
                    d=DH_rtb[i, 1],         # Link offset d (m)
                    a=DH_rtb[i, 2],         # Link length a (m)
                    alpha=DH_rtb[i, 3],     # Link twist alpha (rad)
                    jointtype='R',          # *** ASSUMING REVOLUTE ***
                    m=M[i],                 # Link mass (kg)
                    r=CenterOfMass[i],      # Center of mass vector [x, y, z] (m)
                    I=InertiaMatrices[i]    # Link inertia tensor (3x3) (kg.m^2)
                    # Add friction, motor inertia etc. if available: Jm, B, G, Tc
                )
                links.append(link)
        robot_rtb = rtb.DHRobot(links, name="FreeCAD_RTB_Robot")
        G = [0, 0, -9.81]

        # Example results using roboticstoolbox
        print("  ")
        print("Joint torques RTB")
        print("q_dot = 0, q_ddot = 0")
        displayMatrix(robot_rtb.rne(q, np.zeros_like(robot.Angles), np.zeros_like(robot.Angles), gravity=G))
        print("q_dot = 10, q_ddot = 0")
        displayMatrix(robot_rtb.rne(q, np.ones_like(robot.Angles)*10, np.zeros_like(robot.Angles), gravity=G))
        print("q_dot = 0, q_ddot = 10")
        displayMatrix(robot_rtb.rne(q, np.zeros_like(robot.Angles), np.ones_like(robot.Angles)*10, gravity=G))


    def IsActive(self):        
        return True

FreeCADGui.addCommand('printDynamics', printDynamics())
