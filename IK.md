## Assignment: Inverse Kinematics + Pose GUI + Trajectory for 6-DOF Arm (ROS 2 Jazzy & Gazebo Harmonic)

### 0) Overview

Extend the FK + slider assignment by implementing **inverse kinematics
(IK)** for the same 6-DOF robot arm.

You will:

-   Take a **desired end-effector (EE) pose** (position + orientation),
-   Compute a **joint configuration** that realizes that pose,
-   Send those joint angles to the existing
    **JointGroupPositionController**, and\
-   Provide a simple **pose GUI** (or RViz interface) to move the EE
    around in 3D space.

You may choose either:

-   An **analytic IK** (if your arm is "nice" enough), or\
-   A **numerical IK** method (e.g., Jacobian pseudo-inverse / damped
    least squares).

------------------------------------------------------------------------

### 1) Learning Objectives

By the end, you should be able to:

-   Formulate **IK** for a 6-DOF serial manipulator using your existing
    **DH model / FK function**.
-   Implement a **numerical solver loop** (or an analytic solver) that
    converges to a valid joint solution.
-   Integrate IK with **ros2_control** and your existing **FK + TF**
    pipeline.
-   Work with **EE pose targets** (geometry_msgs/PoseStamped) and
    trajectory interpolation for smooth motion.
-   Reason about **joint limits, singularities, and unreachable
    targets**.

------------------------------------------------------------------------

### 2) Prerequisites

You should already have from the previous assignment:

-   **rk_ws** with **rk_demo** package.
-   6-DOF arm spawned in **Gazebo Harmonic** and controllable via
    `JointGroupPositionController`.
-   Working **FK node** that publishes `base_link → ee_link` TF.
-   Basic Python & ROS 2 (rclpy, tf2, Tkinter or RViz2, etc.).

New packages you'll use:

-   `geometry_msgs`
-   `tf_transformations` or your own helpers for RPY/quaternion
    conversions.
-   Optionally `interactive_markers` or RViz if you choose that path.

------------------------------------------------------------------------

### 3) Provided / Expected Skeleton

You will **extend** the existing structure:

    rk_ws/
      src/
        rk_demo/
          rk_demo/
            __init__.py
            fk_node.py
            slider_gui.py
            pid_follower.py
            ik_node.py
            pose_gui.py
            traj_node.py
          launch/
            bringup.launch.py
            ik_demo.launch.py
          config/
            controllers.yaml
          urdf/
            arm.urdf.xacro
          package.xml
          setup.cfg
          setup.py

You must add `ik_node.py`, `pose_gui.py`, and **register them** in
`setup.py` under `console_scripts`.

------------------------------------------------------------------------

### 4) Tasks & Checkpoints

#### Task A --- Prep & Refactor FK

1.  Refactor FK logic into:
    -   `rk_demo/kinematics.py`
2.  Confirm FK with TF.

**Checkpoint A:** FK matches TF within numerical tolerance.

------------------------------------------------------------------------

#### Task B --- IK Formulation

Choose one:

1.  **Numerical IK (recommended):**
    -   Pseudo-inverse or DLS:
        -   `dq = J⁺ * error`
        -   or `dq = Jᵀ (J Jᵀ + λ² I)⁻¹ * error`
2.  **Analytic IK (optional).**

Handle:

-   Joint limits
-   Singularities
-   Non-convergence

**Checkpoint B:** `ik()` returns `(success, q_solution, err)`.

------------------------------------------------------------------------

#### Task C --- IK Node (`ik_node.py`)

-   Subscribes: `/joint_states`, `/rk_demo/ee_target`
-   Runs IK on new targets
-   Publishes solved joint array to the position controller

**Checkpoint C:** Publishing a `PoseStamped` moves the arm.

------------------------------------------------------------------------

#### Task D --- Pose GUI (`pose_gui.py`)

GUI with sliders:

-   x, y, z
-   roll, pitch, yaw
-   "Send" + "Home"

Publishes `/rk_demo/ee_target`.

**Checkpoint D:** GUI pose maps correctly to arm in Gazebo.

------------------------------------------------------------------------

#### Task E --- Trajectory Node (Optional)

Generate smooth joint-space interpolation from current → IK solution.

**Checkpoint E:** Smooth, non-jerky motion.

------------------------------------------------------------------------

#### Task F --- Launch Integration

`ik_demo.launch.py` should bring up:

-   Gazebo-spawned robot
-   Controllers
-   `ik_node`
-   GUI
-   (Optional) FK node + RViz

**Checkpoint F:**\
Single command launches the whole demo.

------------------------------------------------------------------------

### 5) Deliverables

-   Code: `kinematics.py`, `ik_node.py`, `pose_gui.py`, optionally
    `traj_node.py`
-   Launch file: `ik_demo.launch.py`
-   Updated `setup.py`
-   Readme explaining IK method, limitations, how to run
-   Short demo video/GIF

------------------------------------------------------------------------

### 6) Checklist

-   Clean build
-   IK solutions valid
-   GUI intuitive
-   Smooth motion
-   Clear documentation
