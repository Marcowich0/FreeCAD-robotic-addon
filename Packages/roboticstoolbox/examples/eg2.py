import roboticstoolbox as rtb
robot = rtb.models.URDF.Puma560()
robot.plot(robot.qn)