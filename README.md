**4-DOF RRPP Manipulator Simulator with Moving Boxes**

**Overview**

This Python project simulates a 4-DOF RRPP (Revolute-Revolute-Prismatic-Prismatic) robotic manipulator performing pick-and-place tasks with movable boxes. The simulator visualizes the robot, its end-effector trajectory, and interactive boxes on a shelf with multiple slabs. 

**Features:**

-   **4-DOF RRPP Kinematics:** Implements Forward Kinematics (FK) and a fast, constrained Inverse Kinematics (IK) solver (analytic θ<sub>1</sub>, scalar search for θ<sub>2</sub>). 

-   **Pick and Place Task Simulation:** The robot executes a sequence of pick-and-place tasks, moving boxes from the floor to specified shelf locations. 

-   **Dynamic Objects:** Simulated blue boxes are linked to the end-effector when **"grasped"** and snap accurately onto the shelf slabs when **"released"**. 

-   **Trajectory Visualization:** Displays the end-effector (EE) position and a fading, dotted trajectory line. 

-   **Scene Components:** Includes a floor grid, a shelf structure, and a moving status label. 

**Robot Kinematics and DH Parameters**

The robot is a 4-DOF RRPP manipulator, meaning: 

-   **Joint 1 -- Revolute (Yaw):** The robot rotates horizontally around the vertical z-axis. 

-   **Joint 2 -- Revolute (Pitch):** The arm swings up/down in a vertical plane. 

-   **Joint 3 -- Prismatic (Linear Extension):** The arm extends forward/backward. 

-   **Joint 4 -- Prismatic (Wrist Extension):** The last link extends to reach precise depths. 

This combination is designed for planar reaching, shelf insertion, and pick-and-place tasks. 

**DH Parameters Table**

The Denavit-Hartenberg (DH) convention is used to model the links and joints. The FK formulation in the code corresponds to the parameters below, where L1 is the base height, L2 is the shoulder link length, and d<sub>3</sub> and d<sub>4</sub> are the prismatic joint variables. 

| Axis(i) | Type |  aᵢ  | αᵢ (deg) |   dᵢ   | θᵢ (deg) | Joint Variable        |
|---------|------|------|----------|--------|-----------|------------------------|
|    1    |  R   |  0   |   -90    |  L₁    |   θ₁     | θ₁ (Yaw)              |
|    2    |  R   |  0   |    90    |   0    |   θ₂     | θ₂ (Pitch)            |
|    3    |  P   |  L₂  |    0     |  d₃    |    0     | d₃ (Extension)        |
|    4    |  P   |  0   |    0     |  d₄    |    0     | d₄ (Extension)        |

Where: 
-   L₁ -- base height 

-   L₂ -- shoulder link length 

-   d₃ -- horizontal sliding extension 

-   d₄ -- vertical extension 

These parameters are used to compute the transformation matrices and final end-effector pose. 

**Major Components**

① **Base Frame and Joint 1 (Yaw Rotation)**

-   The robot is mounted on the floor, with the z-axis upward. 

-   Joint 1 rotates the entire upper arm left/right. 

-   This rotation determines the horizontal direction of the end-effector. 

② **Shoulder Link (L2) and Joint 2 (Pitch)**

-   Joint 2 rotates in a vertical plane. 

-   It lifts or lowers the arm relative to the ground. 

-   Combined with Joint 1, it defines the robot's 3D workspace cone. 

③ **Prismatic Joint 3 --- Main Linear Extension (d3)**

-   This joint extends the arm outward horizontally. 

-   It behaves like a telescoping beam. 

-   Used to reach deeper into the shelf while maintaining a stable posture. 

④ **Prismatic Joint 4 --- Wrist Extension (d4)** 

-   Provides the fine positioning needed to:\
    ✓ insert a box precisely\
    ✓ pull back smoothly\
    ✓ avoid collisions with shelves 

⑤ **End-Effector (EE) and Gripper Proxy**

-   Shown as a small cube (the robot does not simulate fingers). 

-   During "grasp", a box becomes linked to the EE coordinates. 

-   During "release", the box "unparents" and snaps onto shelf surface. 

**Forward Kinematics (FK)**

FK computes the position and orientation of the end-effector from given joint values. 

How FK is applied: 

1.  Rotate around z-axis by θ<sub>1</sub> 

2.  Tilt upward/downward by θ<sub>2</sub> 

3.  Translate along the axis by d<sub>3</sub> 

4.  Translate again along the same axis by d<sub>4</sub> 

Together, FK builds the full chain: 

T = T1(θ<sub>1</sub>) * T2(θ<sub>2</sub>) * T3(d<sub>3</sub>) * T4(d<sub>4</sub>) 

This determines: 

-   Where the arm is 

-   Where the EE is 

-   Where the box will be if grasped 

**Inverse Kinematics (IK)**

IK is used when the robot knows the desired EE position and must compute joint values: 

1.  Compute θ<sub>1</sub> from target X,Y (horizontal direction). 

1.  Compute θ<sub>2</sub> using a constrained-search method for stability. 

1.  Compute d<sub>3</sub> so the arm extends to correct radial distance. 

1.  Compute d<sub>4</sub> to set the precise depth needed for shelf insertion. 

This makes the robot reliable for shelf tasks. 

**Box Movement and Attachment Logic**

**Grasp Phase**

-   A boolean flag links the box to the EE. 

-   Box position = FK(EE). 

**Move Phase**

-   The box follows the EE trajectory exactly. 

-   Dotted line shows the EE path. 

**Release Phase**

1.  Compute correct slab surface height: 

z = slab_base + box_height/2 

1.  Snap the box onto the slab. 

1.  Break the parent link. 

**End-Effector Trajectory (Dotted Curve)**

The dotted trajectory line shows: 

-   Smooth cubic interpolation in joint space 

-   EE path generated through numeric FK evaluation 

-   Fading effect: older points lighter, new ones darker 

It helps visualize: 

-   Motion planning 

-   Shelf insertion 

-   Return path to pick another box 

**Whole System Working**

The flow of the simulation: 

1.  Load shelf 

2.  Generate pick points on floor 

3.  Generate place points on slab centers 

For each task: 

1.  IK → reach floor 

2.  Attach box 

3.  IK → move to slab 

4.  Release 

5.  Return home 

6.  Animate FK frames and trajectory 

**Prerequisites**

To run this simulation, you need Python and the following libraries: 

-   numpy 

-   matplotlib 

-   Scipy 

**GitHub Repository**

Full Project source code: <https://github.com/Erum330/Erum330-Robot_Manipulation_FinalCW.git> 

**Presentation Link**

Youtube Link: <https://youtu.be/8QtGPwK8Q3w> 
