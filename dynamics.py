from main_utils import get_robot, vec_to_numpy, mat_to_numpy, numpy_to_mat, np_rotation, numpy_to_rotation, np_translation, displayMatrix
from secondary_utils import partial_derivative
from forward_kinematics import getDHTransformations, getJacobianCenter
import numpy as np
import sympy as sp
import re



def defineCenterOffMass():
    robot = get_robot()
    com = []
    for link, dh in zip(robot.Links, robot.DHLocalCoordinateSystems):
        ol_A_rc = np.array([*vec_to_numpy(link.LinkedObject.Shape.CenterOfMass),1])
        ol_A_dh = mat_to_numpy(dh.Placement.Matrix)
        r_c = (np.linalg.inv(ol_A_dh) @ ol_A_rc)[:3]
        com.append(r_c/1000)
    print("Center of mass for each link")
    print(com)
    robot.CenterOfMass = com
        


##################################################################################################


def updateMomentOfInertia():
    robot = get_robot()
    updateMasses()

    inertia_list = []
    for link, local_dh, mass in zip(robot.Links, robot.DHLocalCoordinateSystems, robot.Masses):
        I_global_mm2 = mat_to_numpy(link.Shape.MatrixOfInertia)[:3, :3]
        I_global_SI = mass * I_global_mm2/link.Shape.Mass * (10**-6)
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

def computeJointTorques(q=None, q_dot=None, q_ddot = None):
    robot = get_robot()
    q = np.zeros(len(robot.VariablesSympy)) if q is None else np.array(q).astype(float)
    q_dot = np.zeros(len(robot.VariablesSympy)) if q_dot is None else np.array(q_dot).astype(float)
    q_ddot = np.zeros(len(robot.VariablesSympy)) if q_ddot is None else np.array(q_ddot).astype(float)

    gravity = np.array([0, 0, -9.81])
    M = robot.Masses

    def D(q):
        Jac_center_full = getJacobianCenter(q, SI=True)
        Jac_vci = [jac[:3,:] for jac in Jac_center_full]
        Jac_wi = [jac[3:,:] for jac in Jac_center_full]
        I_global = [R[:3,:3] @ I @ R[:3,:3].T for R, I in zip(getDHTransformations(q, SI=True), robot.InertiaMatrices)]
        D = sum([m_i * Jac_vci.T @ Jac_vci + Jac_wi.T @ J_i @ Jac_wi for m_i, Jac_vci, Jac_wi, J_i in zip(M, Jac_vci, Jac_wi, I_global)])
        return np.array(D)
    
    def C(q, q_dot):
        C = np.zeros((len(q), len(q)))
        D_num_diff = [partial_derivative(D, q, i) for i in range(len(q))]
        for k in range(len(q)):
            for j in range(len(q)):
                C[k,j] = sum(1/2 * (D_num_diff[i][k,j] + D_num_diff[j][k,i] - D_num_diff[k][i,j]) * q_dot[i] for i in range(len(q)))
        return C
    
    def P(q):
        rc_global = [(o_A_ol @ np.array([*r_c,1]))[:3] for o_A_ol,  r_c in zip(getDHTransformations(q, SI=True), robot.CenterOfMass)]
        P = sum([m_i * gravity.T @ r_c for m_i, r_c in zip(M, rc_global)], np.array([0]))
        return P
    
    def g(q):
        return np.array([partial_derivative(P, q, i) for i in range(len(q))]).reshape(-1)

    g_vec = g(q)
    C_mat = C(q, q_dot)
    D_mat = D(q)

    tau = D_mat @ q_ddot + C_mat @ q_dot + g_vec

    #print("Gravitational term")
    #displayMatrix(g_vec)
    #print("Dampening term")
    #displayMatrix(C_mat)
    #print("Mass term")
    #displayMatrix(D_mat)
    #print("Joint torques")
    #displayMatrix(tau)

    return tau
    
