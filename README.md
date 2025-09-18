# Assignment: Forward Kinematics + Slider GUI + Simple PID in ROS 2 Jazzy & Gazebo Harmonic

## 0) Overview
Implement forward kinematics (FK) for a 6‑DOF robot arm, spawn it in **Gazebo Harmonic (gz‑sim v8+)**, control joints via a **JointGroupPositionController**, and create a **slider GUI** that publishes target joint positions. Optionally wire up a minimal **PID follower** and visualize the end‑effector (EE) pose over TF.

---

## 1) Learning Objectives
By the end, you should be able to:
- Integrate a 6‑DOF URDF/SDF with **ros2_control** in Gazebo Harmonic.
- Compute FK using a DH table (or your model’s parameters) and publish `base_link → ee_link` via TF.
- Command joints with a position controller and a simple GUI of sliders.
- (Optional) Implement a basic position PID and understand controller alternatives.

---

## 2) Prerequisites
- ROS 2 **Jazzy** installed and sourced.
- Gazebo **Harmonic** (gz‑sim v8+) + `ros_gz`.
- A working 6‑DOF robot model (**URDF/Xacro** or **SDF**).
- Python basics (rclpy, Tkinter).
- Packages: `robot_state_publisher`, `controller_manager`, `ros2_control`, `ros_gz_sim`, `xacro`, `tf2_ros`.

---

## 3) Provided Skeleton (You may adapt)
You will create a workspace and a Python package containing:
- URDF/Xacro with `ros2_control` section (or SDF with the appropriate Gazebo plugin).
- Controller YAML for `JointStateBroadcaster` + `JointGroupPositionController`.
- A **bringup** launch file to spawn into Gazebo and load controllers.
- `fk_node.py`: publishes `base_link → ee_link` using FK.
- `slider_gui.py`: six sliders → target positions.
- `pid_follower.py` (optional): simple PID loop.

Suggested structure:
```text
rk_ws/
  src/
    rk_demo/
      rk_demo/
        __init__.py
        fk_node.py
        slider_gui.py
        pid_follower.py      # optional
      launch/
        bringup.launch.py
      config/
        controllers.yaml
      urdf/
        arm.urdf.xacro       # or your SDF under models/
      package.xml
      setup.cfg
      setup.py
```

---

## 4) Tasks & Checkpoints

### Task A — Workspace & Package
1. Create workspace and package:
   ```bash
   mkdir -p ~/rk_ws/src && cd ~/rk_ws
   rosdep update
   cd src
   ros2 pkg create --build-type ament_python rk_demo --dependencies \
     rclpy sensor_msgs std_msgs geometry_msgs tf2_ros urdf xacro \
     control_msgs controller_manager_msgs
   ```
2. Add files: `urdf/arm.urdf.xacro`, `config/controllers.yaml`, `launch/bringup.launch.py`, and Python nodes under `rk_demo/rk_demo`.
3. Register entry points in `setup.py` (`console_scripts`).

**Checkpoint A:** `colcon build` completes; package is discoverable.

---

### Task B — Robot Model + ros2_control
1. In your URDF/Xacro (or SDF), declare six actuated joints `joint1..joint6`.
2. Add a `ros2_control` block compatible with **gz_ros2_control** (position command & position/velocity state). Example (URDF/Xacro excerpt):
   ```xml
   <ros2_control name="gazebo_system" type="system">
     <hardware>
       <plugin>gz_ros2_control/GazeboSystem</plugin>
     </hardware>
     <joint name="joint1">
       <command_interface name="position"/>
       <state_interface name="position"/>
       <state_interface name="velocity"/>
     </joint>
     <!-- repeat for joint2..joint6 -->
   </ros2_control>
   ```
3. Ensure joint names match across **URDF/SDF**, **controllers.yaml**, and all nodes.

**Checkpoint B:** `robot_state_publisher` runs without errors; joint names appear in `/joint_states` (after controllers load).

---

### Task C — Controllers & Gazebo Bringup
1. Create `config/controllers.yaml`:
   ```yaml
   controller_manager:
     ros__parameters:
       update_rate: 250  # Hz

       joint_state_broadcaster:
         type: joint_state_broadcaster/JointStateBroadcaster

       position_controller:
         type: position_controllers/JointGroupPositionController

   position_controller:
     ros__parameters:
       joints: [joint1, joint2, joint3, joint4, joint5, joint6]
       command_interfaces: [position]
       state_interfaces: [position, velocity]
   ```
2. Write `launch/bringup.launch.py` to:
   - Start `robot_state_publisher` with your URDF/Xacro (or SDF bridge).
   - Launch `ros2_control_node` with `controllers.yaml` and a `robot_description` parameter.
   - Spawn the model into Gazebo via `ros_gz_sim create` (or include SDF directly).
   - Load/activate both controllers:
     ```bash
     ros2 control load_controller --set-state active joint_state_broadcaster
     ros2 control load_controller --set-state active position_controller
     ```

**Checkpoint C:**  
Run:
```bash
gz sim -v 4 -r empty.sdf
# separate terminal:
source ~/rk_ws/install/setup.bash
ros2 launch rk_demo bringup.launch.py
ros2 control list_controllers
```
Both controllers should be **active**.

---

### Task D — Forward Kinematics Node
1. Implement `rk_demo/rk_demo/fk_node.py`:
   - Subscribe to `/joint_states`.
   - Map positions to `joint1..joint6`.
   - Use your DH table (or URDF-derived transforms) to compute the 4×4 pose.
   - Publish `base_link → ee_link` as a TF transform.

2. Validate in RViz2 (optional):
   ```bash
   rviz2
   ```
   Add TF; verify `ee_link` moves when joints move.

**Checkpoint D:** TF shows `ee_link` transform updating when joint targets change.

---

### Task E — Slider GUI Publisher
1. Implement `rk_demo/rk_demo/slider_gui.py` (Tkinter):
   - Six horizontal sliders in range `[-3.14, 3.14]` (or per your joint limits).
   - Publish a `Float64MultiArray` on `/rk_demo/target_positions` at ~25 Hz.
2. Wire to controller:
   - Either publish directly to `/position_controller/commands` (array of 6 positions), **or**
   - Publish to `/rk_demo/target_positions` and forward with a node (Task F).

**Checkpoint E:** Moving sliders changes joint angles in Gazebo.

---

### Task F — (Optional) Minimal PID Follower
1. Implement `rk_demo/rk_demo/pid_follower.py`:
   - Subscribe to `/joint_states` and `/rk_demo/target_positions`.
   - Compute `e, ∫e, de/dt`; (for simplicity) forward targets to the `JointGroupPositionController`.
   - If you want *your* PID to be authoritative, switch to `forward_command_controller/JointGroupCommandController` (effort interface) and publish torques instead.
2. Tune `kp, ki, kd`; add output limiting and anti‑windup if using effort mode.

**Checkpoint F:** Joints track slider targets smoothly without oscillation.

---

### Task G — Build & Run (End‑to‑End)
1. Build:
   ```bash
   cd ~/rk_ws
   rosdep install --from-paths src -y --rosdistro jazzy
   colcon build
   source install/setup.bash
   ```
2. Run:
   ```bash
   # Terminal 1
   gz sim -v 4 -r empty.sdf

   # Terminal 2
   ros2 launch rk_demo bringup.launch.py

   # Terminal 3
   ros2 run rk_demo fk_node
   ros2 run rk_demo slider_gui
   # optional
   ros2 run rk_demo pid_follower
   ```

**Final Checkpoint:**  
- `/joint_states` updating  
- `/position_controller/commands` receiving arrays  
- TF publishes `base_link → ee_link`  
- The arm in Gazebo moves with sliders

---

## 5) Deliverables
1. **Code package** `rk_demo` (all sources).
2. **Readme (1–2 pages)** explaining:
   - FK approach (DH parameters or alternative),
   - Controller configuration,
   - How to run (commands),
   - Known limitations and next steps.
3. **Demo video/GIF (≤90s)** showing:
   - Gazebo arm, GUI sliders, TF in RViz2 (or printed EE pose),
   - Smooth tracking with PID (if implemented).

---

## 6) Checklist
- **Build & Launch Success:** Clean build, controllers active, model spawns.
- **FK Correctness:** `ee_link` pose consistent with known joint configs (sanity checks).
- **Control Integration:** Sliders drive joints; stable behavior; reasonable ranges.
- **Code Quality:** Clear structure, parameterization, comments, `setup.py` entry points.
- **Documentation:** Readme completeness and clarity.
- **Optional PID:** Demonstrated tuning and explanation (or effort‑mode variant).

---

## 7) Stretch Goals (choose any)
- Replace DH with URDF‑derived kinematics (parse tree, accumulate transforms).
- Switch to **effort** interface + `JointGroupCommandController` and implement proper PID with anti‑windup & output limits.
- Add joint limit enforcement and soft‑stops.
- Show EE pose numerically in the GUI; add a “home” button.
- Replace Tkinter with an RViz2 panel or rqt plugin.
- Record a rosbag; plot tracking error.

---

## 8) Troubleshooting Checklist
- **Nothing moves:** `ros2 control list_controllers` → both **active**? Joint names match? Topic is correct (`/position_controller/commands`)?
- **No `/joint_states`:** Broadcaster not loaded/active; check controller_manager logs.
- **Plugin not found:** Verify `gz_ros2_control` installed and environment sourced.
- **Exploding robot:** Check inertias and damping; reduce gains or limit slider ranges.
- **FK looks wrong:** Joint sign/order mismatches; confirm DH convention and offsets.

---

## 9) Submission
- Zip: `rk_demo_<yourname>.zip` with code + readme + short demo.
- Include a one‑liner on tested environment (OS, Gazebo Harmonic version, ROS 2 Jazzy).
