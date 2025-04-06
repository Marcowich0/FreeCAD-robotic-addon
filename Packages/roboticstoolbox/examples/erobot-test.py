import roboticstoolbox as rtb 

robot = rtb.models.URDF.PR2()

print(robot)
print(robot.ee_links[0])
robot.hierarchy()
# print(robot.ets())

print(rtb.ETS.SE3(robot.base))
