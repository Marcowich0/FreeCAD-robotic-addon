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
        com.append(r_c/1000)
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
import sympy as sp
from sympy.utilities.lambdify import lambdify

import sys
sys.setrecursionlimit(10_000)



def defineTauSympy():
    print("Starting computation of joint torques")
    robot = get_robot()
    q_sym = robot.VariablesSympy
    q_dot_sym = [sp.symbols(f"q_dot_{i+1}") for i, _ in enumerate(robot.Links[1:])]
    q_ddot_sym = [sp.symbols(f"q_ddot_{i+1}") for i, _ in enumerate(robot.Links[1:])]

    N = len(robot.Links)-1

    m = robot.Masses
    g = sp.Matrix([0, 0, -9.81])
    rc_global = [sp.Matrix((o_A_ol * sp.Matrix([*r_c,1]))[:3]) for o_A_ol,  r_c in zip(robot.DHTransformationsSympy, robot.CenterOfMass)]
    Jac_vci = [jac[:3,:] for jac in robot.JacobianCenterSympy]
    Jac_wi = [jac[3:,:] for jac in robot.JacobianCenterSympy]
    I_global = [R[:3,:3] * sp.Matrix(I) * R[:3,:3].T for R, I in zip(robot.DHTransformationsSympy, robot.InertiaMatrices)]

    print("Computing gravitational term")
    P = sum([m_i * g.T * r_c for m_i, r_c in zip(m, rc_global)], sp.Matrix([0]))
    g_vec = sp.Matrix([P.diff(theta) for theta in q_sym])
    print("Gravitational term sympy")
    print(g_vec.subs({theta:0 for theta in q_sym}))


    print("Computing Mass term")
    D = sum(((m_i * Jac_vci.T * Jac_vci + Jac_wi.T * J_i * Jac_wi) for m_i, Jac_vci, Jac_wi, J_i in zip(m, Jac_vci, Jac_wi, I_global)), sp.zeros(len(q_sym), len(q_sym)))
    print("Computing Dampening term")
    C = sp.Matrix([[0 for _ in range(N)] for _ in range(N)])   
    for k in range(N):
        for j in range(N):
            C[k,j] = sum(1/2 * (D[k,j].diff(q_sym[i]) + D[k,i].diff(q_sym[j]) - D[i,j].diff(q_sym[k])) * q_dot_sym[i] for i in range(N))
            print(f"Progress of C: {k*N+j}/{N*N}")
    
    tau = D * sp.Matrix(q_ddot_sym) + C * sp.Matrix(q_dot_sym) + g_vec
    
    print("Saving tau function")
    robot.TauSympy = tau
    print("Done")

    




def computeJointTorques(q=None, q_dot=None, q_ddot = None):
    robot = get_robot()
    q = np.zeros(len(robot.VariablesSympy)) if q is None else np.array(q).astype(float)
    q_dot = np.zeros(len(robot.VariablesSympy)) if q_dot is None else np.array(q_dot).astype(float)
    q_ddot = np.zeros(len(robot.VariablesSympy)) if q_ddot is None else np.array(q_ddot).astype(float)

    gravity = np.array([0, 0, -9.81])
    M = robot.Masses

    def partial_derivative(f, q, i, h=1e-6):
        q_plus = q.copy()
        q_minus = q.copy()
        q_plus[i] += h
        q_minus[i] -= h
        return (f(q_plus) - f(q_minus)) / (2 * h)    

    def D(q):
        Jac_vci = [jac(*q)[:3,:] for jac in robot.JacobianCenterLambdified]
        Jac_wi = [jac(*q)[3:,:] for jac in robot.JacobianCenterLambdified]
        I_global = [R(*q)[:3,:3] @ I @ R(*q)[:3,:3].T for R, I in zip(robot.DHTransformationsLambdified, robot.InertiaMatrices)]
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
        rc_global = [(o_A_ol(*q) @ np.array([*r_c,1]))[:3] for o_A_ol,  r_c in zip(robot.DHTransformationsLambdified, robot.CenterOfMass)]
        P = sum([m_i * gravity.T @ r_c for m_i, r_c in zip(M, rc_global)], np.array([0]))
        return P
    
    def g(q):
        return np.array([partial_derivative(P, q, i) for i in range(len(q))]).reshape(-1)
    
    

    print(f"Shapes of q, q_dot, q_ddot: {q.shape}, {q_dot.shape}, {q_ddot.shape}")

    g_vec = g(q)
    C_mat = C(q, q_dot)
    D_mat = D(q)

    print(f"Shapes of g_vec, C_mat, D_mat: {g_vec.shape}, {C_mat.shape}, {D_mat.shape}")
    print("Gravitational term")
    displayMatrix(g_vec)
    print("Dampening term")
    displayMatrix(C_mat)
    print("Mass term")
    displayMatrix(D_mat)
    tau = D_mat @ q_ddot + C_mat @ q_dot + g_vec
    print("Joint torques")
    displayMatrix(tau)
    return tau
    


    





















def defineSympyTransformations():
    robot = get_robot()

    theta_arr = [sp.symbols(f"theta_{i+1}") for i, _ in enumerate(robot.Links[1:])]
    robot.VariablesSympy = theta_arr

    last = None
    T_arr = [sp.Matrix( mat_to_numpy(robot.Links[0].Placement.Matrix) @ mat_to_numpy(robot.DHCoordinateSystems[0].Placement.Matrix))]
    for dh_perameters, theta in zip(robot.DHPerameters, theta_arr):
        _ , d, a, alpha = dh_perameters
        d, a  = d/1000, a/1000
        dh_trans = sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
                             [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
                             [0, sp.sin(alpha), sp.cos(alpha), d],
                             [0, 0, 0, 1]])
        last = last * dh_trans if last is not None else dh_trans
        T_arr.append(last)
    robot.DHTransformationsSympy = T_arr
    robot.DHTransformationsLambdified = [sp.lambdify(theta_arr, sp.Matrix(transformation), 'numpy') for transformation in T_arr]


def defineSympyJacobian():
    robot = get_robot()
    T_arr = robot.DHTransformationsSympy

    Jac_arr = []
    for i, link in zip(range(1, len(T_arr)), robot.Links[1:]):
        Jac = []
        On = sp.Matrix((T_arr[i] * sp.Matrix([ *robot.CenterOfMass[i] , 1 ]))[:3])

        for j in range(1, len(T_arr)):
            if j <= i:
                Zi = T_arr[j-1][0:3, 2]
                Oi = T_arr[j-1][0:3, 3]

                Jv = Zi.cross((On - Oi))
                Jw = Zi
            else:
                Jv = sp.Matrix([0,0,0])
                Jw = sp.Matrix([0,0,0])
            
            Jac.append([*Jv, *Jw])
        Jac_arr.append(sp.Matrix(Jac).T)
        print("Jacobian for link center of mass", i)
    robot.JacobianCenterSympy = Jac_arr
    robot.JacobianCenterLambdified = [sp.lambdify(robot.VariablesSympy, Jac, 'numpy') for Jac in Jac_arr]