from matplotlib import colors
from matplotlib import cm

def linkcolormap(self, linkcolors="viridis"):  # self is a robot object

    if isinstance(linkcolors, list) and len(linkcolors) == self.n:
        # provided a list of color names
        return colors.ListedColormap(linkcolors)
    else:
        # assume it is a colormap name
        return cm.get_cmap(linkcolors, 6)

import roboticstoolbox as rtb 
robot = rtb.models.DH.Puma560()

c = linkcolormap(robot)
for i in range(6):
    print(c(i))

c = linkcolormap(robot, "inferno")
for i in range(6):
    print(c(i))
print(type(c))
c = linkcolormap(robot, ['red', (0,1,0), 'b', 'yellow', 'cyan', 'brown'])
for i in range(6):
    print(c(i))

print(c(range(6)))
print(type(c))