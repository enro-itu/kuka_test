# 🤖 Assignment — 2-Finger Parallel Gripper for KUKA in Gazebo

## 🎯 Goal
Extend the existing **KUKA robot model in Gazebo** with a **2-finger parallel gripper** that can open/close via ROS2 control (or your control stack) and successfully grasp simple objects (cube/cylinder).

---

## ✅ Learning Objectives
- Model a **mechanical 2-finger gripper** in URDF/Xacro or SDF
- Attach it correctly to the KUKA end-effector
- Configure joints + transmissions + controllers
- Tune physics to allow stable grasping

---

# 🏗 Phase 1 — 2-Finger Gripper Modeling

### 1.1 Structure
Create:
- `gripper_base_link`
- `finger_left_link`
- `finger_right_link`

Joints:
- `finger_left_joint`
- `finger_right_joint`

Symmetry:
- Either mimic joint OR identical control on both

### 1.2 Attachment
Attach `gripper_base_link` to KUKA tool flange (`tool0` / `ee_link`).

### 1.3 Physics
- Add visual + collision
- Add realistic inertia + mass
- Stable damping

**Deliverables**
- `gripper_2finger.urdf.xacro`
- `kuka_with_2finger_gripper.urdf.xacro`
- Screenshot in Gazebo

---

# ⚙ Phase 2 — Control

Use either:
- ROS2 Control (recommended)
- Gazebo plugin
- Custom controller

Must support:
- open command
- close command

Provide:
- config yaml
- launch file
- example commands

---

# 🧪 Phase 3 — Grasping Test

Test with:
- cube / cylinder object

Procedure:
1. Open gripper
2. Align with object
3. Close slowly
4. Lift
5. Release

Provide:
- video or gif

---

# 🔍 Phase 4 — Validation

Explain:
- design choices
- control method
- physics tuning
- results + limits

---

# 📂 Submission Structure
```
kuka_2finger_gripper_assignment_<name>/
 ├── urdf_or_sdf/
 ├── models/
 ├── config/
 ├── launch/
 ├── video/
 └── report.md
```

---

# ⭐ Bonus
- rubber pads
- force-based gripping
- pick & place
- MoveIt integration

---

# ✅ Checklist
- gripper attaches
- fingers move
- opens & closes
- grasps object
- stable simulation
- proper docs

Good luck 😎
