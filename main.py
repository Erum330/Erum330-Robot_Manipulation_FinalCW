# sim_step7.py
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ---------------- Shelf ----------------
class Shelf:
    def __init__(self, slab_heights, width=1.0, depth=0.4):
        self.slab_heights = slab_heights
        self.width = width
        self.depth = depth

    def get_slab_position(self, index):
        if 0 <= index < len(self.slab_heights):
            return self.slab_heights[index]
        return None

    def draw(self, ax):
        # Draw simple slabs as flat bars
        for z in self.slab_heights:
            ax.bar3d(0, 0, z, self.width, self.depth, 0.02, color='sienna', alpha=0.8)

# ---------------- Robot ----------------
class Robot:
    def __init__(self):
        self.joint_angles = np.zeros(4)  # 4-DOF RRPP
        print("Robot initialized with 4-DOF.")

    def fk(self, joint_angles):
        # Simplified FK: return 4 points (base, joint1, joint2, EE)
        theta1, theta2, d3, d4 = joint_angles
        L1 = 0.45
        L2 = 0.35
        P0 = np.array([0,0,0])
        P1 = np.array([0,0,L1])
        P2 = np.array([L2*np.cos(theta2)*np.cos(theta1),
                       L2*np.cos(theta2)*np.sin(theta1),
                       L1 + L2*np.sin(theta2)])
        x_sh = L2*np.cos(theta2) + d3
        P4 = np.array([x_sh*np.cos(theta1),
                       x_sh*np.sin(theta1),
                       L1 + L2*np.sin(theta2) + d4])
        return P0, P1, P2, P4

    def ik(self, target_pos):
        # Placeholder IK: returns zeros
        return np.zeros(4)

# ---------------- Simulation ----------------
class Simulation:
    def __init__(self):
        self.shelf = Shelf(slab_heights=[0.2, 0.6, 1.0], width=1.0, depth=0.4)
        self.robot = Robot()
        self.tasks = self.build_tasks()

    def build_tasks(self):
        pick_points = [
            np.array([0.45, 0.18, 0.05]),
            np.array([0.25, -0.35, 0.05]),
            np.array([0.60, 0.35, 0.05])
        ]
        place_points = [
            np.array([0.9, 0.08, 0.2]),
            np.array([0.82, 0.15, 0.6]),
            np.array([0.78, 0.22, 1.0])
        ]
        return list(zip(pick_points, place_points))

    def joint_trajectory(self, start, end, steps=10):
        start = np.array(start)
        end = np.array(end)
        return [start*(1-t) + end*t for t in np.linspace(0,1,steps)]

    def draw_box(self, ax, position, size=0.07):
        x, y, z = position
        ax.bar3d(x - size/2, y - size/2, z, size, size, size, color='deepskyblue')

    def run(self):
        fig = plt.figure(figsize=(10,8))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim([-0.5,1.5])
        ax.set_ylim([-0.5,1.5])
        ax.set_zlim([0,1.8])
        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')

        # Draw shelf
        self.shelf.draw(ax)

        # Draw boxes at pick and place positions
        for pick, place in self.tasks:
            self.draw_box(ax, pick)
            self.draw_box(ax, place)

        # Draw robot at initial pose
        P0, P1, P2, P4 = self.robot.fk(self.robot.joint_angles)
        ax.plot([P0[0], P1[0]], [P0[1], P1[1]], [P0[2], P1[2]], 'k-', lw=5)
        ax.plot([P1[0], P2[0]], [P1[1], P2[1]], [P1[2], P2[2]], 'b-', lw=5)
        ax.plot([P2[0], P4[0]], [P2[1], P4[1]], [P2[2], P4[2]], 'r-', lw=4)
        ax.scatter([P4[0]], [P4[1]], [P4[2]], color='magenta', s=80, marker='X')

        plt.title("Step 7: 3D Robot & Shelf Visualization")
        plt.show()

# ---------------- Main ----------------
if __name__ == "__main__":
    sim = Simulation()
    sim.run()
