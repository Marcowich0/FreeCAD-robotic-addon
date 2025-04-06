import matplotlib.pyplot as plt
from matplotlib import patches
import numpy as np
from spatialmath import SE2

fig, ax = plt.subplots()

h = 0.3
t = 0.8  # start of head taper
c = 0.5  # centre x coordinate
w = 1    # width in x direction

# coords = np.array([
#     [-c,     h],
#     [t - c,  h],
#     [w - c,  0],
#     [t - c, -h],
#     [-c,    -h],
# ])

coords = np.array([
    [-c,  h],
    [ w,  0],
    [-c, -h],
])

arr_img = plt.imread('examples/car1.png', format='png')

# from matplotlib.offsetbox import (OffsetImage,AnnotationBbox)
# imagebox = OffsetImage(arr_img, zoom=0.3)
# imagebox.image.axes = ax

# xy = [0, 0]
# # ab = AnnotationBbox(imagebox, xy,
# #                     xybox=(120., -10.),
# #                     xycoords='data',
# #                     boxcoords="offset points",
# #                     pad=0.5,
# #                     bboxprops =dict(edgecolor='red'),
# #                     )

# # xy is the point being annotated (irrelevant here), and xycooords is 
# # the position of the annotation box
# ab = AnnotationBbox(imagebox, xy,
#                     xybox=(0, 0.),
#                     xycoords='data',
#                     boxcoords="data",
#                     pad=0.5,
#                     frameon=False,
#                     bboxprops =dict(edgecolor='red'),
#                     )
                    
# print(dir(ab))
# ab.set_transform()
# ax.add_artist(ab)

import matplotlib.transforms as mtransforms

def imshow_affine(ax, z, *args, **kwargs):
    im = ax.imshow(z, *args, **kwargs)
    x1, x2, y1, y2 = im.get_extent()
    im._image_skew_coordinate = (x2, y1)
    return im

xi = 0
yi = 0
deg = 45
width, height = 0.5, 0.5
im = imshow_affine(ax, arr_img, interpolation='none',
                    extent=[0, width, 0, height], clip_on=True,
                    alpha=0.8)
center_x, center_y = width//2, height//2
im_trans = (mtransforms.Affine2D()
            .rotate_deg_around(center_x, center_y, deg)
            .translate(xi, yi)
            + ax.transData)
im.set_transform(im_trans)


# color is fillcolor + edgecolor
# facecolor if None is default
p = patches.Polygon(coords, edgecolor='r', facecolor=None, fill=True)
ax.add_patch(p)

ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
plt.grid(True)
# plt.axis('equal')

for theta in np.linspace(0, 2 * np.pi, 20):
    xy = SE2(0, 0, theta) * coords.T
    p.set_xy(xy.T)
    plt.pause(0.1)


