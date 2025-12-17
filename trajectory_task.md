# Assignment: Trajectory Planning for a Robot Arm (ROS 2 + Gazebo)

## 0) Context (what exists already)
You are given a ROS 2 Python package named **`rk_demo`** that includes:
- A robot arm model in **Xacro/URDF** (`robot_arm.urdf.xacro`)
- RViz visualization launch (`display.launch.py`)
- Gazebo spawn launch (`gz.launch.py`)
- A basic test launch (`test.launch.py`)
- Standard ROS 2 dependencies for control, TF, messages, and Gazebo sim integration

You also mention that the **workspace envelope is already calculated briefly**. This assignment turns that into a usable constraint for planning.

---

## 1) Goal
Implement a **trajectory planning pipeline** for the robot arm that can:
- Move the end-effector from a start pose to a goal pose
- Generate **smooth, time-parameterized** joint trajectories
- Respect **joint limits**, **velocity/acceleration limits**, and the provided **workspace envelope**
- Execute trajectories in **Gazebo** (and optionally visualize in RViz)

---

## 2) What you must deliver
1. A planning node (Python) inside `rk_demo` (example: `rk_demo/trajectory_planner.py`)
2. A launch file to run the planner with the sim (example: `launch/plan_and_execute.launch.py`)
3. A short `README.md` that explains:
   - how to launch Gazebo / RViz
   - how to send goals
   - what topics/services/actions are used
   - how envelope constraints are applied
4. Demo evidence:
   - a short screen recording **or**
   - screenshots + log output showing the tests below passing

---

## 3) Core requirements

### A) Trajectory types (choose one primary, implement one secondary)
**Primary (must):**
- Joint-space **quintic polynomial** trajectory (minimum-jerk style) between waypoints

**Secondary (choose one):**
- Trapezoidal / S-curve time-scaling
- Cartesian straight-line path (task-space) + IK at samples
- Sampling-based planner (very simple RRT in joint space is OK if documented)

---

### B) Time parameterization (must)
Your output trajectory must include timestamps such that:
- Joint velocities stay under limits
- Joint accelerations stay under limits
- Trajectory is continuous (no jumps)

You may implement:
- Analytical time scaling (trapezoid/S-curve), or
- Iterative scaling until constraints are met

---

### C) Envelope constraints (must)
Use the existing “envelope” concept to reject or reroute motion:
- If a waypoint (or sampled Cartesian point) violates the envelope → **planner must refuse** or **adapt** (e.g., add intermediate waypoint).
- Document the envelope representation you use:
  - e.g., AABB box, cylinder, convex hull, spherical shell, etc.

**Acceptance rule**
- The planner must never command a trajectory that is known to violate the envelope (by your own checker).

---

### D) ROS 2 interface (must)
Provide at least one of the following external interfaces:

**Option 1 (recommended):** Action server
- `/plan_and_execute` action taking goal pose or joint goal
- returns success/failure + a message

**Option 2:** Service
- `/plan_trajectory` service returning a `trajectory_msgs/JointTrajectory`

**Option 3:** Topic-based
- subscribe `/goal_pose` and publish `/planned_trajectory`

Also: publish a status topic (e.g., `/planner/status`) with useful strings.

Package already depends on `control_msgs`, `controller_manager_msgs`, `geometry_msgs`, etc., so pick interfaces that match these dependencies. fileciteturn2file0

---

## 4) Suggested architecture

### Step 1 — Modeling & limits extraction
- Parse the URDF to obtain:
  - joint names in order
  - joint limits (position, velocity if available)
- Store limits internally and validate all solutions.

### Step 2 — Planning (path)
- Build a path in either:
  - joint space, or
  - Cartesian space (then run IK to get joint samples)

### Step 3 — Smoothing & timing
- Generate a smooth curve between points
- Apply time scaling (respect vel/acc limits)

### Step 4 — Execution
- Publish / send to controller:
  - `trajectory_msgs/JointTrajectory` (preferred), or
  - whatever controller interface you are using in your setup

---

## 5) Mandatory tests (what you must show working)

### Test 1 — Joint-space point-to-point
- Start at a known joint configuration (home)
- Plan to a target joint configuration
- Execute smoothly in Gazebo (no instant jumps)

### Test 2 — Envelope boundary test
- Send a goal that is *outside* the envelope
- Planner must reject it with a clear reason

### Test 3 — Multi-waypoint trajectory
- Plan through **3 waypoints** (including an intermediate one you choose)
- Show continuous motion and correct timing

### Test 4 — Limit stress test
- Pick a “fast” goal
- Show that time-scaling increases duration to keep within constraints

---

## 6) Grading rubric
- Trajectory correctness (smoothness + continuity)
- Time parameterization (vel/acc limits enforced)
- Envelope constraint handling (reject/adapt correctly)
- ROS 2 integration & usability (action/service/topic + logs)
- Code quality + README + reproducibility

---

## 7) Implementation notes aligned with your repo
- Your package is **ament_python** and already installs launch + URDF resources. fileciteturn2file0 fileciteturn2file1
- You already have:
  - RViz visualization launch (`display.launch.py`)
  - Gazebo spawn launch (`gz.launch.py`)
  - A minimal test launch (`test.launch.py`)
- Your deliverable should add a **new planner node** + a **combined launch** that starts sim + planner.

---

## 8) Bonus (optional)
Pick any two:
- Obstacle avoidance (simple spheres/boxes) on top of envelope
- Trajectory visualization markers in RViz (`visualization_msgs/MarkerArray`)
- Online replanning (stop + replan on new goal)
- Jerk-limited S-curve time scaling

---

**Outcome:** After this assignment, the robot arm should move like a “real” robot: planned paths, smooth timing, and safety constraints—rather than direct joint teleop.
