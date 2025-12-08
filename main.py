import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from math import cos, sin, atan2, pi, asin

# ---------------- Robot Parameters ----------------
L1 = 0.55       # base vertical offset
L2 = 0.50       # shoulder link length
TOOL_LEN = 0.12 # wrist/tool length

THETA1_MIN, THETA1_MAX = -pi, pi
THETA2_MIN, THETA2_MAX = -pi/2, pi/2
D3_MIN, D3_MAX = -0.25, 0.80
THETA4_MIN, THETA4_MAX = -pi, pi

IK_TOL = 2e-3
BOX_SIZE = 0.07

# ---------------- Kinematics ----------------
def fk_full(joint):
    theta1, theta2, d3, theta4 = joint
    P0 = np.array([0.0,0.0,0.0])
    P1 = np.array([0.0,0.0,L1])
    P2 = np.array([L2*cos(theta2)*cos(theta1),
                   L2*cos(theta2)*sin(theta1),
                   L1 + L2*sin(theta2)])
    x_sh = L2*cos(theta2) + d3
    z_sh = L1 + L2*sin(theta2)
    P3 = np.array([x_sh*cos(theta1), x_sh*sin(theta1), z_sh])
    elev = theta2 + theta4
    tool_dir = np.array([cos(theta1)*cos(elev), sin(theta1)*cos(elev), sin(elev)])
    P4 = P3 + TOOL_LEN*tool_dir
    return P4, (P0,P1,P2,P3,P4)

def inverse_kinematics(tool_tip):
    x, y, z_tip = tool_tip
    wrist_z = z_tip + TOOL_LEN
    s = (wrist_z - L1)/L2
    s = np.clip(s, -1.0, 1.0)
    theta2 = asin(s)
    r = np.hypot(x, y)
    d3 = r - L2*cos(theta2)
    d3 = float(np.clip(d3, D3_MIN, D3_MAX))
    theta1 = atan2(y, x)
    theta1 = np.clip(theta1, THETA1_MIN, THETA1_MAX)
    theta4 = -pi/2 - theta2
    joint = np.array([theta1, theta2, d3, theta4])
    P4, _ = fk_full(joint)
    if np.linalg.norm(P4 - np.array([x, y, z_tip])) > IK_TOL:
        raise ValueError(f"IK residual too large: {np.linalg.norm(P4 - np.array([x, y, z_tip]))}")
    return joint

def joint_traj(start, end, steps=60):
    start, end = np.array(start), np.array(end)
    return [start*(1-t)+end*t for t in np.linspace(0,1,steps)]

# ---------------- Scene ----------------
def draw_box(ax, pos, size=BOX_SIZE, color='deepskyblue'):
    x,y,z = pos
    return ax.bar3d(x-size/2, y-size/2, z, size, size, size, color=color)

def draw_shelf(ax, origin=(0.6,0.3,0.3), width=0.4, depth=0.3, levels=3, thickness=0.02):
    x0, y0, z0 = origin
    level_h = 0.25
    slab_centers = []
    for i in range(levels):
        z = z0 + i*level_h
        ax.bar3d(x0, y0, z, width, depth, thickness, color='wheat', alpha=0.8)
        slab_centers.append(z + thickness)
    post_positions = [
        (x0, y0),
        (x0+width, y0),
        (x0, y0+depth),
        (x0+width, y0+depth)
    ]
    post_height = z0 + levels*level_h + thickness
    for px, py in post_positions:
        ax.plot([px, px], [py, py], [0, post_height], color='saddlebrown', lw=2)
    return slab_centers

def draw_floor_grid(ax, limit=1.2, spacing=0.2):
    for x in np.arange(-limit, limit+0.01, spacing):
        ax.plot([x,x],[-limit,limit],[0,0], color='lightgray', lw=0.5)
    for y in np.arange(-limit, limit+0.01, spacing):
        ax.plot([-limit,limit],[y,y],[0,0], color='lightgray', lw=0.5)

def draw_work_envelope(ax, radius=0.9, height=1.5, alpha=0.1):
    u = np.linspace(0, 2*np.pi, 60)
    v = np.linspace(0, pi/2, 30)
    X = radius*np.outer(np.cos(u), np.sin(v))
    Y = radius*np.outer(np.sin(u), np.sin(v))
    Z = height*np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(X,Y,Z, color='lightgreen', alpha=alpha, zorder=0)

# ---------------- Tasks ----------------
def build_tasks(shelf_centers):
    pick_points = [
        np.array([0.3,  0.0, 0.04]),
        np.array([0.25, -0.2, 0.04]),
        np.array([0.35,  0.2, 0.04])
    ]
    place_points = [
        np.array([0.65, 0.45, shelf_centers[0]]),
        np.array([0.65, 0.45, shelf_centers[1]]),
        np.array([0.65, 0.45, shelf_centers[2]])
    ]
    return [(p, pl) for p, pl in zip(pick_points, place_points)]

# ---------------- Animation ----------------
def animate_scene_with_labels():
    fig = plt.figure(figsize=(14,10))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-1,1]); ax.set_ylim([-1,1]); ax.set_zlim([0,1.5])
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.set_zlabel('Z [m]')
    ax.set_title("Pick and Place Robot Simulation", fontsize=16, fontweight='bold')

    draw_floor_grid(ax)
    shelf_centers = draw_shelf(ax)
    draw_work_envelope(ax)

    tasks = build_tasks(shelf_centers)
    pose_pairs=[]
    home = np.array([0.0, -0.4,0.5])
    q_home = inverse_kinematics(home)
    for pick, place in tasks:
        q_pick = inverse_kinematics(pick)
        q_place = inverse_kinematics(place)
        pose_pairs.append((q_pick, q_place))

    full=[]; labels=[]; current=q_home
    full += joint_traj(current, pose_pairs[0][0], 50)
    labels += ["Move from home"]*50
    current = pose_pairs[0][0]

    for idx,(qpick,qplace) in enumerate(pose_pairs):
        full+=joint_traj(current,qpick,40); labels+=[f"Approach pick {idx+1}"]*40
        full+=[qpick]*10; labels+=[f"Grasp {idx+1}"]*10
        full+=joint_traj(qpick,qplace,60); labels+=[f"Move to place {idx+1}"]*60
        full+=[qplace]*10; labels+=[f"Release {idx+1}"]*10
        if idx<len(pose_pairs)-1:
            next_pick = pose_pairs[idx+1][0]
            full+=joint_traj(qplace,next_pick,40); labels+=[f"Transit to pick {idx+2}"]*40
            current = next_pick

    box_positions=[p.copy() for p,_ in tasks]
    holding=[False]*len(box_positions)
    box_artists=[]

    colors=['red','blue','green','orange']
    line01,line12,line23,line34=[ax.plot([],[],[],lw=3,color=colors[i])[0] for i in range(4)]
    joint0,joint1,joint2,joint3,ee=ax.scatter([],[],[],s=60,color='k'),ax.scatter([],[],[],s=60,color='k'),ax.scatter([],[],[],s=60,color='k'),ax.scatter([],[],[],s=60,color='k'),ax.scatter([],[],[],s=80,color='magenta')

    ee_traj = []  # wrist trajectory
    phase_label = ax.text2D(0.02, 0.95, "", transform=ax.transAxes, fontsize=14, color='purple', fontweight='bold')

    def update(i):
        q = full[i]; P4,(P0,P1,P2,P3,_)=fk_full(q)
        line01.set_data([P0[0],P1[0]],[P0[1],P1[1]]); line01.set_3d_properties([P0[2],P1[2]])
        line12.set_data([P1[0],P2[0]],[P1[1],P2[1]]); line12.set_3d_properties([P1[2],P2[2]])
        line23.set_data([P2[0],P3[0]],[P2[1],P3[1]]); line23.set_3d_properties([P2[2],P3[2]])
        line34.set_data([P3[0],P4[0]],[P3[1],P4[1]]); line34.set_3d_properties([P3[2],P4[2]])

        joint0._offsets3d=([P0[0]],[P0[1]],[P0[2]])
        joint1._offsets3d=([P1[0]],[P1[1]],[P1[2]])
        joint2._offsets3d=([P2[0]],[P2[1]],[P2[2]])
        joint3._offsets3d=([P3[0]],[P3[1]],[P3[2]])
        ee._offsets3d=([P4[0]],[P4[1]],[P4[2]])

        # Boxes
        for idx,(p,pl) in enumerate(tasks):
            if f"Grasp {idx+1}" in labels[i]: holding[idx]=True
            if f"Release {idx+1}" in labels[i]: holding[idx]=False; box_positions[idx]=pl.copy()
            if holding[idx]: box_positions[idx]=P4.copy()
        for art in box_artists: art.remove()
        box_artists.clear()
        for pos in box_positions: box_artists.append(draw_box(ax,pos))

        # Phase label
        phase_label.set_text(labels[i])

        # Continuous fading wrist trajectory
        ee_traj.append(P4.copy())
        if len(ee_traj) > 50: ee_traj.pop(0)
        if hasattr(update, "traj_line"): update.traj_line.remove()
        traj_array = np.array(ee_traj)
        update.traj_line, = ax.plot(traj_array[:,0], traj_array[:,1], traj_array[:,2],
                                    color='magenta', alpha=0.5, lw=3)

    # Legend
    ax.plot([],[],[],color='red', lw=3,label='Base→Shoulder')
    ax.plot([],[],[],color='blue', lw=3,label='Shoulder→Elbow')
    ax.plot([],[],[],color='green', lw=3,label='Elbow→Wrist')
    ax.plot([],[],[],color='orange', lw=3,label='Wrist→Tool')
    ax.scatter([],[],[],s=80,color='magenta', label='End-Effector')
    ax.legend(loc='upper right')

    ani=FuncAnimation(fig, update, frames=len(full), interval=40, blit=False)
    plt.show()

if __name__=="__main__":
    animate_scene_with_labels()
