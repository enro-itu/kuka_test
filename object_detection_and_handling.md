# Assignment: RealSense-Based Pick & Hold System (ROS2 Jazzy)

---

## Replace These Placeholders (One-Time Setup)
Use these placeholders everywhere until real names are known.

### Robot frames
- `<BASE_FRAME>` = `base_link`
- `<EE_FRAME>` = `ee_link`
- `<TCP_FRAME>` = `tcp_link`  
  *(Tool center point between gripper fingers)*

### RealSense frames
- `<CAMERA_LINK>` = `camera_link`
- `<CAMERA_COLOR_OPTICAL>` = `camera_color_optical_frame`
- `<CAMERA_DEPTH_OPTICAL>` = `camera_depth_optical_frame`

### MoveIt / controllers
- `<ARM_GROUP>` = `arm`
- `<GRIPPER_GROUP>` = `gripper`
- `<ARM_TRAJ_CONTROLLER>` = `arm_controller`
- `<GRIPPER_CONTROLLER>` = `gripper_controller`

### Topics
- `<RGB_TOPIC>` = `/camera/color/image_raw`
- `<DEPTH_TOPIC>` = `/camera/aligned_depth_to_color/image_raw`
- `<CAMERA_INFO>` = `/camera/color/camera_info`

---

## Goal (Definition of Done)
- Detect a target object using RealSense RGB + depth
- Estimate **3D pose** (position + orientation if possible)
- Transform pose into `<BASE_FRAME>`
- Plan and execute **pregrasp → grasp → lift** with MoveIt2
- Close 2‑finger gripper and **hold** the object

Success target: ≥ 7/10 successful grasps on tabletop objects.

---

## PHASE 0 — Repository Setup (1–2 hours)

### Task 0.1: Create packages
Create these ROS2 packages (or folders):
1. `arm_bringup` – launch files, params
2. `arm_moveit_config` – MoveIt2 configuration
3. `arm_perception` – perception → pose
4. `arm_manipulation` – grasp logic & state machine

**Deliverable:** Repository structure exists.

### Task 0.2: Documentation
Create:
- `docs/frames.md`
- `docs/topics.md`

List all placeholders above.

---

## PHASE 1 — Robot Bringup & Controllers (½ day)

### Task 1.1: robot_state_publisher
- Launch `robot_state_publisher` with URDF/Xacro
- Visualize in RViz

**Acceptance check**
- TF contains `<BASE_FRAME>` and `<EE_FRAME>`
- `/joint_states` topic exists

**Deliverable:** `arm_bringup/launch/robot.launch.py`

### Task 1.2: ros2_control
Configure and launch:
- `joint_state_broadcaster`
- `joint_trajectory_controller` (`<ARM_TRAJ_CONTROLLER>`)
- Optional gripper controller (`<GRIPPER_CONTROLLER>`)

**Acceptance check**
- Controller accepts trajectory commands

**Deliverable:** `arm_bringup/config/ros2_controllers.yaml`

---

## PHASE 2 — MoveIt2 Setup (1 day)

### Task 2.1: Create MoveIt2 config
- Planning group: `<ARM_GROUP>`
- End effector: `<GRIPPER_GROUP>` (recommended)
- End‑effector frame: `<TCP_FRAME>`
- IK plugin: KDL

**Deliverable:** `arm_moveit_config` package

### Task 2.2: Controller mapping
Map MoveIt2 to:
- `<ARM_TRAJ_CONTROLLER>`
- `<GRIPPER_CONTROLLER>` (if used)

**Acceptance check**
- Plan + execute to random pose in RViz

### Task 2.3: Named poses
Add SRDF named states:
- `home`
- `pregrasp_ready`
- `carry`

**Deliverable:** Updated SRDF + `docs/named_poses.md`

---

## PHASE 3 — RealSense Integration (2–4 hours)

### Task 3.1: Launch RealSense
- Launch `realsense2_camera`
- Ensure depth aligned to color

**Acceptance check**
- `<RGB_TOPIC>` publishes
- `<DEPTH_TOPIC>` publishes
- Camera TF frames visible

**Deliverable:** `arm_bringup/launch/realsense.launch.py`

---

## PHASE 4 — Camera Mounting Modes (1 day)

### Mode A: Fixed Camera

**Task 4A.1** Publish static TF `<BASE_FRAME> → <CAMERA_LINK>`
- Use placeholder translation and rotation

**Deliverable:** `arm_bringup/launch/tf_fixed_camera.launch.py`

### Mode B: Eye‑in‑Hand Camera

**Task 4B.1** Publish static TF `<EE_FRAME> → <CAMERA_LINK>`

**Deliverable:** `arm_bringup/launch/tf_eye_in_hand.launch.py`

**Acceptance check (both modes)**
- TF tree correct in RViz

---

## PHASE 5 — Perception → Pose (1–3 days)

### Task 5.1: Standard outputs
Publish:
- `/target_pose_cam` (`PoseStamped`, frame `<CAMERA_LINK>`)
- `/target_pose_base` (`PoseStamped`, frame `<BASE_FRAME>`)
- `/target_marker` (RViz visualization)

**Deliverable:** `arm_perception/target_pose_publisher`

### Method 1 (MVP): AprilTag
**Task 5.2** Detect AprilTag and estimate full pose

**Deliverable:** `arm_perception/apriltag_pose_node`

### Method 2 (General objects)
**Task 5.3** 2D detection (YOLO or similar)

**Task 5.4** Depth → 3D centroid
- Median depth in mask/bbox
- Back‑project using camera intrinsics

**Task 5.5** Orientation estimate
- PCA on object point cloud
- Extract yaw for gripper alignment
- Fallback to default orientation if unreliable

**Deliverable:** `arm_perception/detector_depth_pose_node`

---

## PHASE 6 — Grasp Pose Generation (1 day)

### Task 6.1: Grasp parameters
Create `arm_manipulation/config/grasp_params.yaml`:
- `tcp_frame: <TCP_FRAME>`
- `base_frame: <BASE_FRAME>`
- `approach_distance: <APPROACH_M>`
- `grasp_distance: <GRASP_M>`
- `lift_distance: <LIFT_M>`
- `top_grasp: true`
- `default_gripper_yaw: <YAW_RAD>`
- Gripper open/close placeholders

### Task 6.2: Pose computation
Compute:
- `pregrasp_pose`
- `grasp_pose`
- `lift_pose`

Rules:
- Prefer top grasps for tabletop objects
- Align yaw if orientation available

**Deliverable:** `arm_manipulation/grasp_pose_generator`

---

## PHASE 7 — Pick & Hold State Machine (1–2 days)

### Task 7.1: State machine
Implement node `pick_and_hold_node` with states:
1. OPEN_GRIPPER
2. GO_PREGRASP_READY
3. WAIT_TARGET
4. PLAN_TO_PREGRASP
5. EXECUTE_PREGRASP
6. APPROACH_CARTESIAN
7. CLOSE_GRIPPER
8. LIFT_CARTESIAN
9. GO_CARRY
10. HOLD

**Deliverable:** `arm_manipulation/pick_and_hold_node`

---

## PHASE 8 — Planning Scene Safety (½ day)

### Task 8.1: Collision objects
- Add table as collision box
- Optional: attach object after grasp

**Deliverable:** Planning scene helper module

---

## PHASE 9 — Visual Servo Upgrade (Optional)

### Task 9.1: Final approach correction
- Continuously update `/target_pose_base`
- Apply small XY corrections until within tolerance
- Then descend and close gripper

**Deliverable:** `arm_manipulation/visual_servo_module`

---

## PHASE 10 — Full System Launch (2–4 hours)

### Task 10.1: Master launch
Create `pick_system.launch.py` launching:
- Robot bringup
- ros2_control
- MoveIt2
- RealSense
- Static TF (argument: `camera_mode:=fixed|eye_in_hand`)
- Perception node
- Manipulation node
- RViz config

**Deliverable:** One command starts full system

---

## Final Acceptance Test
1. Launch full system
2. Place object on table
3. `/target_pose_base` visible and correct in RViz
4. Run pick state machine
5. Arm grasps, lifts, and holds object for ≥ 5 s

---

## Notes
- All names are placeholders and must be replaced later
- Focus first on AprilTag‑based grasping for fastest success
- Add general object grasping and visual servoing afterward

