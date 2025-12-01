import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

BOX_SIZE = 0.07

def draw_shelf(ax, origin=(0.75, 0, 0), width=0.5, depth=0.35, height=1.5, levels=3, color='sienna'):
    x0, y0, z0 = origin
    level_h = height / levels
    # Poles
    pole_th = 0.03
    corners = [(x0, y0), (x0 + width, y0), (x0, y0 + depth), (x0 + width, y0 + depth)]
    for cx, cy in corners:
        ax.bar3d(cx, cy, z0, pole_th, pole_th, height, color=color, alpha=0.9)
    # Slabs
    for i in range(levels + 1):
        z = z0 + i * level_h - pole_th/2
        ax.bar3d(x0, y0, z, width, depth, pole_th, color=color, alpha=0.8)
    slab_base = [z0 + i*level_h for i in range(levels)]
    return level_h, slab_base

def draw_box(ax, pos, size=BOX_SIZE, color='deepskyblue'):
    x, y, z = pos
    return ax.bar3d(x - size/2, y - size/2, z - size/2, size, size, size, color=color)

# Scene
fig = plt.figure(figsize=(10,8))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([0, 1.5]); ax.set_ylim([-0.75, 0.75]); ax.set_zlim([0, 2])
ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')

level_h, slab_base = draw_shelf(ax)
boxes_pos = [[0.8, 0.1, 0.04], [0.85, -0.1, 0.04], [0.9, 0, 0.04]]
boxes_artists = [draw_box(ax, pos) for pos in boxes_pos]

plt.show()
