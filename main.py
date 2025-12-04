import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.animation import FuncAnimation
from math import cos, sin, atan2, radians, pi
from scipy.optimize import minimize_scalar
from collections import deque

# ---------------- Robot Parameters ----------------
L1 = 0.45        # base height (m)
L2 = 0.35        # shoulder link length (m)

# Joint limits
THETA1_MIN, THETA1_MAX = -pi, pi
THETA2_MIN, THETA2_MAX = radians(-90), radians(90)
D3_MIN, D3_MAX = 0.05, 1.2
D4_MIN, D4_MAX = -0.4, 0.8

IK_POS_TOL = 8e-4  # m tolerance

BOX_SIZE = 0.07    # Box side length (m)

# ---------------- Kinematics ----------------
def fk_full(joint):
    theta1, theta2, d3, d4 = joint
    P0 = np.array([0.0, 0.0, 0.0])
    P1 = np.array([0.0, 0.0, L1])
    P2 = np.array([L2 * cos(theta2) * cos(theta1),
                   L2 * cos(theta2) * sin(theta1),
                   L1 + L2 * sin(theta2)])
    x_sh = (L2 * cos(theta2) + d3)
    P4 = np.array([x_sh * cos(theta1), x_sh * sin(theta1), L1 + L2 * sin(theta2) + d4])
    return P4, (P0, P1, P2, P4)

def reachable_by_geometry(target):
    x, y, z = target
    r = np.hypot(x, y)
    max_r = L2 + D3_MAX
    min_r = max(0.0, L2 + D3_MIN)
    min_z = L1 + D4_MIN
    max_z = L1 + L2 + D4_MAX
    return (r <= max_r + 1e-6) and (r >= min_r - 1e-6) and (z >= min_z - 1e-6) and (z <= max_z + 1e-6)

def inverse_kinematics(target):
    x, y, z = target
    theta1 = atan2(y, x)
    r = np.hypot(x, y)

    if not reachable_by_geometry(target):
        raise ValueError("Target is outside quick geometric bounds.")

    def cost(th2):
        d3 = r - L2 * cos(th2)
        d4 = z - L1 - L2 * sin(th2)
        pen = 0.0
        if d3 < D3_MIN: pen += 1e3 * (D3_MIN - d3) ** 2
        if d3 > D3_MAX: pen += 1e3 * (d3 - D3_MAX) ** 2
        if d4 < D4_MIN: pen += 1e3 * (D4_MIN - d4) ** 2
        if d4 > D4_MAX: pen += 1e3 * (d4 - D4_MAX) ** 2
        x_c = (L2 * cos(th2) + d3) * cos(theta1)
        y_c = (L2 * cos(th2) + d3) * sin(theta1)
        z_c = L1 + L2 * sin(th2) + d4
        err = np.sqrt((x - x_c) ** 2 + (y - y_c) ** 2 + (z - z_c) ** 2)
        return err + pen

    res = minimize_scalar(cost, bounds=(THETA2_MIN, THETA2_MAX), method='bounded', options={'xatol':1e-6})
    th2 = float(res.x)
    d3 = r - L2 * cos(th2)
    d4 = z - L1 - L2 * sin(th2)
    joint = np.array([theta1, th2, float(np.clip(d3, D3_MIN, D3_MAX)), float(np.clip(d4, D4_MIN, D4_MAX))], dtype=float)
    pos, _ = fk_full(joint)
    if np.linalg.norm(pos - np.array(target)) > IK_POS_TOL:
        raise ValueError("IK residual too large")
    return joint

# ---------------- Trajectory helpers ----------------
def lerp(a, b, t):
    return a * (1 - t) + b * t

def joint_traj(start, end, steps=60):
    start = np.array(start)
    end = np.array(end)
    return [lerp(start, end, s) for s in np.linspace(0, 1, steps)]

# ---------------- Scene visuals ----------------
def draw_shelf(ax, origin=(0.75, 0.0, 0.0), shelf_width=0.5, shelf_depth=0.35, shelf_height=1.5, levels=3, color='sienna'):
    x0, y0, z0 = origin
    pole_th = 0.03
    level_h = shelf_height / levels
    corners = [(x0, y0), (x0 + shelf_width, y0), (x0, y0 + shelf_depth), (x0 + shelf_width, y0 + shelf_depth)]
    for (cx, cy) in corners:
        ax.bar3d(cx, cy, z0, pole_th, pole_th, shelf_height, color=color, alpha=0.9, shade=True)
    for i in range(levels + 1):
        z = z0 + i * level_h - pole_th / 2.0
        ax.bar3d(x0, y0, z, shelf_width, shelf_depth, pole_th, color=color, alpha=0.8, shade=True)
    return pole_th, level_h

def draw_box(ax, pos, size=BOX_SIZE, color='deepskyblue', edgecolor='k'):
    x, y, z = pos
    bars = ax.bar3d(x - size/2, y - size/2, z - size/2, size, size, size,
                    color=color, edgecolor=edgecolor, alpha=1.0)
    return bars

def draw_floor_grid(ax, plot_limit=1.2, spacing=0.2):
    xs = np.arange(-plot_limit, plot_limit + 1e-6, spacing)
    ys = np.arange(-plot_limit, plot_limit + 1e-6, spacing)
    for x in xs:
        ax.plot([x, x], [-plot_limit, plot_limit], [0, 0], color='lightgray', linewidth=0.5, alpha=0.6)
    for y in ys:
        ax.plot([-plot_limit, plot_limit], [y, y], [0, 0], color='lightgray', linewidth=0.5, alpha=0.6)

# ---------------- Tasks ----------------
def build_tasks():
    pick_points = [
        np.array([0.45,  0.18, 0.05]),
        np.array([0.25, -0.35, 0.05]),
        np.array([0.60,  0.35, 0.05])
    ]
    place_points = [
        np.array([0.9,  0.08, 0.4]),
        np.array([0.82, 0.15, 0.95]),
        np.array([0.78, 0.22, 1.45])
    ]
    return [(p, pl) for p, pl in zip(pick_points, place_points)]

# ---------------- Animation ----------------
def animate_scene(tasks, shelf_origin=(0.75, 0.0, 0.0)):
    pose_pairs = []
    for pick, place in tasks:
        pick_adj = pick.copy(); pick_adj[2] = max(pick_adj[2], 0.04)
        q_pick = inverse_kinematics(pick_adj)
        q_place = inverse_kinematics(place)
        pose_pairs.append((q_pick, q_place))

    full = []; labels = []
    current = pose_pairs[0][0]
    for idx, (qpick, qplace) in enumerate(pose_pairs):
        full += joint_traj(current, qpick, steps=60); labels += [f"Approach pick {idx+1}"]*60
        full += [qpick]*12; labels += [f"Grasp {idx+1}"]*12
        full += joint_traj(qpick, qplace, steps=90); labels += [f"Move to place {idx+1}"]*90
        full += [qplace]*12; labels += [f"Release {idx+1}"]*12
        if idx < len(pose_pairs) - 1:
            next_pick = pose_pairs[idx+1][0]
            full += joint_traj(qplace, next_pick, steps=80); labels += [f"Transit to pick {idx+2}"]*80
            current = next_pick
        else:
            full += joint_traj(qplace, pose_pairs[0][0], steps=100); labels += ["Return to neutral"]*100

    box_positions = [np.array([p[0], p[1], 0.04]) for p, _ in tasks]
    holding = [False]*len(box_positions)
    box_artists = []

    ee_traj_len = 50
    ee_traj = deque(maxlen=ee_traj_len)

    fig = plt.figure(figsize=(13,10))
    ax = fig.add_subplot(111, projection='3d')
    plot_limit = 1.2
    ax.set_xlim([-plot_limit, plot_limit]); ax.set_ylim([-plot_limit, plot_limit]); ax.set_zlim([0, 2.0])
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('RRPP Manipulator with Moving Boxes')

    draw_floor_grid(ax, plot_limit=plot_limit, spacing=0.2)
    pole_th, level_h = draw_shelf(ax, origin=shelf_origin, shelf_width=0.5, shelf_depth=0.35, shelf_height=1.5, levels=3)
    
    # Compute top of each slab for box placement
    slab_base = shelf_origin[2] + np.arange(3) * level_h
    shelf_top_z = slab_base + level_h  # top surface of each slab

    line01, = ax.plot([], [], [], lw=8, color='k', alpha=0.75)
    line12, = ax.plot([], [], [], lw=8, color='royalblue', alpha=0.9)
    line23, = ax.plot([], [], [], lw=6, color='crimson', alpha=0.95)
    ee_scatter = ax.scatter([], [], [], color='magenta', s=120, marker='X')
    ee_traj_line, = ax.plot([], [], [], 'k--', lw=1.5, alpha=0.6, label='EE trajectory')
    label_text = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)
    ax.view_init(elev=28, azim=-60)

    def update(i):
        q = full[i]
        _, (P0, P1, P2, P4) = fk_full(q)

        line01.set_data([P0[0], P1[0]], [P0[1], P1[1]]); line01.set_3d_properties([P0[2], P1[2]])
        line12.set_data([P1[0], P2[0]], [P1[1], P2[1]]); line12.set_3d_properties([P1[2], P2[2]])
        line23.set_data([P2[0], P4[0]], [P2[1], P4[1]]); line23.set_3d_properties([P2[2], P4[2]])
        ee_scatter._offsets3d = ([P4[0]], [P4[1]], [P4[2]])

        ee_traj.append(P4.copy())
        traj_points = np.array(ee_traj)
        ee_traj_line.set_data(traj_points[:,0], traj_points[:,1])
        ee_traj_line.set_3d_properties(traj_points[:,2])

        for idx, (p, pl) in enumerate(tasks):
            if "Grasp" in labels[i] and f"{idx+1}" in labels[i]:
                holding[idx] = True
            if "Release" in labels[i] and f"{idx+1}" in labels[i]:
                holding[idx] = False
                # Snap box on top of slab
                box_positions[idx] = np.array([pl[0], pl[1], shelf_top_z[idx] + BOX_SIZE/2])
            if holding[idx]:
                box_positions[idx] = P4.copy()

        for art in box_artists:
            art.remove()
        box_artists.clear()
        for pos in box_positions:
            art = draw_box(ax, pos, size=BOX_SIZE)
            box_artists.append(art)

        label_text.set_text(f"Step {i+1}/{len(full)}  -  {labels[i]}")
        return line01, line12, line23, ee_scatter, ee_traj_line, label_text, *box_artists

    ani = FuncAnimation(fig, update, frames=len(full), interval=40, blit=False)
    ax.legend(loc='upper right')
    plt.show()

# ---------------- Main ----------------
if __name__ == "__main__":
    tasks = build_tasks()
    print("Tasks (pick -> place):")
    for i, (p, pl) in enumerate(tasks, start=1):
        print(f" Task {i}: pick {p}, place {pl}")
        print("  Quick feasibility:", reachable_by_geometry(p), reachable_by_geometry(pl))
    animate_scene(tasks, shelf_origin=(0.75, 0.0, 0.0))
