from roboticstoolbox import *
from spatialmath import *
from spatialmath.base import *
import numpy as np


robot = models.ETS.Panda()
q0 = [.1, .3, .1, 0, 0, 0, 0]

T0 = robot.fkine(q0)
p0 = T0.t
R0 = SO3(T0)

q = q0
dt = 0.01


# # move at velocity (translational, rotational)
# # rotational velocity is an Eulervector
# v = np.r_[0, 0, -0.1, 0, 0, 0]
# for t in np.arange(0.0, 10.0, dt):

#     p = p0 + v[:3] * t
#     # print(p)
#     R = R0  * SO3.EulerVec(v[3:] * t)

#     T = SE3.Rt(R, p)

#     Tactual = robot.fkine(q)
#     Tactual.printline()

#     delta = Tactual.delta(T)  # Tactual - Tstar  in the Tactual frame

#     qd = np.linalg.pinv(robot.jacobe(q)) @ delta * 0.9

#     q += qd * [0.99, 0.98, 1.01, 1.02, 0.99, 1.02, 0.97] * dt

#     print(p0 - Tactual.t)


Tf = SE3(0.3, 0.1, 0.1) * SE3.Rx(np.pi/2) # hand down

delta_vmax = 0.1  # max translational speed in m/s
delta_rmax = 0.1

for t in np.arange(0.0, 10.0, dt):

    Tactual = robot.fkine(q)
    Tactual.printline()

    delta = Tactual.delta(Tf)  # Tactual - Tstar in the Tactual frame

    if norm(delta) < 1e-3:
        print('done at t=', t)
        break
    print(norm(delta))

    delta *= 10
    print(delta)
    # delta is velocity per tick

    # clip translational velocity
    max = np.abs(delta[:3]).max()
    if max > delta_vmax / dt:
        delta[:3] *= delta_vmax / max / dt

    # clip rotational velocity
    max = np.abs(delta[3:]).max()
    if max > delta_rmax / dt:
        delta[3:] *= delta_rmax / max / dt
    print(delta)
    print()


    qd = np.linalg.pinv(robot.jacobe(q)) @ delta

    q += qd * [0.99, 0.98, 1.01, 1.02, 0.99, 1.02, 0.97] * dt
    


