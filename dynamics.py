from main_utils import get_robot, vec_to_numpy, mat_to_numpy, numpy_to_mat, np_rotation, numpy_to_rotation, np_translation, displayMatrix
import numpy as np
import re



def defineCenterOffMass():
    robot = get_robot()
    com = []
    for link, dh in zip(robot.Links, robot.DHLocalCoordinateSystems):
        ol_A_rc = np.array([*vec_to_numpy(link.LinkedObject.Shape.CenterOfMass),1])
        ol_A_dh = mat_to_numpy(dh.Placement.Matrix)
        r_c = (np.linalg.inv(ol_A_dh) @ ol_A_rc)[:3]
        com.append(r_c)
    print("Center of mass for each link")
    print(com)
    robot.CenterOfMass = com
        


##################################################################################################

def updateJacobian():
    robot = get_robot()
    T_arr = robot.DHTransformations
    Jac = []
    On = T_arr[-1][0:3, 3]
    for i in range(1, len(T_arr)):
        Zi = T_arr[i-1][0:3, 2]
        Oi = T_arr[i-1][0:3, 3]
        Jv = np.cross( Zi , (On - Oi)  )
        Jw = Zi
        Jac.append([*Jv, *Jw])
    robot.Jacobian = np.array(Jac).T


def updateJacobianCenter():
    robot = get_robot()
    T_arr = robot.DHTransformations

    Jac_arr = []
    for i, link in zip(range(1, len(T_arr)), robot.Links[1:]):
        Jac = []
        On = (T_arr[i] @ np.array([ *robot.CenterOfMass[i] , 1 ]))[:3]

        for j in range(1, len(T_arr)):
            if j <= i:
                Zi = T_arr[j-1][0:3, 2]
                Oi = T_arr[j-1][0:3, 3]
                Jv = np.cross( Zi , (On - Oi)  )
                Jw = Zi
            else:
                Jv = np.zeros(3)
                Jw = np.zeros(3)
            Jac.append([*Jv, *Jw])
        Jac_arr.append(np.array(Jac).T)
        print("Jacobian for link center of mass", i)
        displayMatrix(np.array(Jac).T)
    robot.JacobianCenter = Jac_arr

##################################################################################################

def updateMomentOfInertia():
    robot = get_robot()
    updateMasses()

    inertia_list = []
    for link, local_dh, mass in zip(robot.Links, robot.DHLocalCoordinateSystems, robot.Masses):
        I_global_mm2 = mat_to_numpy(link.Shape.MatrixOfInertia)[:3, :3]
        I_global_SI = mass * I_global_mm2 * (10**-6)
        rotation = mat_to_numpy(local_dh.Placement.Matrix)[:3, :3]
        I_dh_SI = rotation @ I_global_SI @ rotation.T

        print("Inertia matrix for link", link.Name)
        displayMatrix(np.round(I_dh_SI,3))
        inertia_list.append(I_dh_SI)

    robot.InertiaMatrices = inertia_list
        

##################################################################################################

def updateMasses():
    robot = get_robot()
    masses = []
    for link in robot.Links:
        rho_text = link.LinkedObject.ShapeMaterial.Properties["Density"]
        match = re.search(r'[-+]?\d*\.\d+e[-+]?\d+|\d+', rho_text)

        if match:
            rho = float(match.group())*(10**9)
        else:
            print("Density not found, assuming steel with density 7900 kg/m^3")
            rho = 7900 
        V = link.Shape.Volume * (10**-9)
        M = rho * V
        masses.append(M)
    robot.Masses = masses



##################################################################################################


def computeJointTorques(q, q_dot, q_ddot):
    robot = get_robot()
    PotentialEnergy_arr = []

    J_vc = robot.Jacobian
    for i, link in enumerate(robot.Links):
        dh = mat_to_numpy(robot.DHTransformations[i])
        r_c = dh @ vec_to_numpy(link.Shape.CenterOfMass)
        mass = robot.Masses[i]
        g = np.array([0, 0, -9.81])

        PotentialEnergy_arr.append( - mass * g @ r_c)

