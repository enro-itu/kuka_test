# ASSIGNMENT --- Workspace Envelope of rk_demo Robot Arm

**Project:** ENRO / rk_demo\
**Assigned to:** Arif\
**Reviewed by:** Emre / Software Team\
**Date:** 09.12.2025

------------------------------------------------------------------------

# 1. Objective

Compute, visualize, and document the **workspace envelope** of the 3‑DOF
robot arm defined in `robot_arm.urdf.xacro` in the `rk_demo` package.

This includes FK implementation, joint limit extraction, sampling,
plotting, optional RViz markers, and a final technical report.

------------------------------------------------------------------------

# 2. Deliverables

### Mandatory

-   `scripts/generate_workspace.py`\
-   `data/workspace_points.csv` (≥ 5000 points)\
-   `figures/workspace.png`\
-   `docs/workspace_report.md`\
-   Clean PR into branch `feature/workspace-envelope-arif`

### Optional

-   RViz MarkerArray workspace visualization\
-   Convex hull estimation

------------------------------------------------------------------------

# 3. Directory Structure (Target)

    rk_demo/
     ├── scripts/
     │    └── generate_workspace.py
     ├── data/
     │    └── workspace_points.csv
     ├── figures/
     │    └── workspace.png
     ├── docs/
     │    └── workspace_report.md
     └── arif/
          └── notes.md   (optional work-in-progress notes)

------------------------------------------------------------------------

# 4. Tasks

## Task 1 --- Extract Joint Limits from URDF (Programmatically)

### Requirements

-   Generate full URDF via:

        ros2 run xacro xacro $(ros2 pkg prefix rk_demo)/share/rk_demo/urdf/robot_arm.urdf.xacro > robot_arm.urdf

-   Parse URDF using `urdfpy` or XML.

-   Extract `<limit lower="…" upper="…">` for all revolute/continuous
    joints.

-   Store in a Python dictionary:

    ``` python
    joint_limits = {
        "joint1": (lower, upper),
        "joint2": (lower, upper),
        "joint3": (lower, upper),
    }
    ```

### TODO for Arif

-   [ ] Write `parse_joint_limits.py` inside your script\
-   [ ] Include joint table inside the report

------------------------------------------------------------------------

## Task 2 --- Implement Forward Kinematics

Write FK using URDF link lengths. FK must match the kinematic chain used
by:

-   `display.launch.py` (RViz model)
-   `gz.launch.py` (Gazebo spawn)

### FK Template (You will complete)

``` python
import numpy as np

def fk(j1, j2, j3, links):
    # links = { "L1": ..., "L2": ..., "L3": ... }

    # TODO: convert joint angles j1, j2, j3 from degrees/radians as needed

    # TODO: implement DH transforms or URDF-based chain matrices

    T = np.eye(4)

    # TODO: fill transforms
    # T = T @ T1 @ T2 @ T3

    x, y, z = T[0,3], T[1,3], T[2,3]
    return x, y, z
```

### TODO for Arif

-   [ ] Implement FK\
-   [ ] Verify FK correctness using RViz visualization

------------------------------------------------------------------------

## Task 3 --- Sample Workspace

### Requirements

-   Use joint limits from Task 1\
-   Steps recommended:
    -   Joint 1: 2--3°\
    -   Joint 2: 1--3°\
    -   Joint 3: 1--3°
-   Generate ≥ 5000 valid FK points

### Sampling Template

``` python
for j1 in np.arange(lim1[0], lim1[1], step1):
    for j2 in np.arange(lim2[0], lim2[1], step2):
        for j3 in np.arange(lim3[0], lim3[1], step3):
            x, y, z = fk(j1, j2, j3, links)
            csv_writer.writerow([x, y, z])
```

### TODO for Arif

-   [ ] Implement sampling\
-   [ ] Save `data/workspace_points.csv`

------------------------------------------------------------------------

## Task 4 --- Plotting the Workspace

### Requirements

-   Use `matplotlib`\
-   3D scatter plot\
-   Color points by height (z-axis)\
-   Save as `figures/workspace.png`

### Template

``` python
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

df = pd.read_csv("data/workspace_points.csv")

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

p = ax.scatter(df.x, df.y, df.z, c=df.z, s=1)

ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_ylabel("Z [m]")

plt.savefig("figures/workspace.png", dpi=300)
```

### TODO for Arif

-   [ ] Produce a clean, readable plot\
-   [ ] Include in report

------------------------------------------------------------------------

## Task 5 --- Optional RViz Visualization

Create a node that publishes MarkerArray for workspace points.

### Template

``` python
from visualization_msgs.msg import Marker, MarkerArray

pub = node.create_publisher(MarkerArray, "/workspace_markers", 10)
```

### TODO (Optional)

-   [ ] Add MarkerArray visualization\
-   [ ] Add new launch file if needed

------------------------------------------------------------------------

# 5. Final Report (workspace_report.md)

Must include:

### 1. System Overview

### 2. URDF Joint Limits Table

### 3. Parameters (sampling resolution, total samples)

### 4. 3D Plot Screenshot

### 5. Observations

-   Max reach\
-   Min/max Z\
-   Dead zones\
-   Shape of envelope

### 6. Recommendations

------------------------------------------------------------------------

# 6. Acceptance Criteria

To mark the assignment **DONE**, deliver:

-   [ ] `generate_workspace.py`\
-   [ ] `workspace_points.csv` ≥ 5000 points\
-   [ ] `workspace.png`\
-   [ ] `workspace_report.md`\
-   [ ] FK validated using RViz model\
-   [ ] Joint limits extracted programmatically\
-   [ ] Clean code (PEP8)

------------------------------------------------------------------------

# 7. Notes for Arif

-   Use the robot model from `robot_arm.urdf.xacro`\
-   Do not modify launch files\
-   Keep everything inside `rk_demo` structure\
-   If stuck → ask Emre or Yusuf

------------------------------------------------------------------------

**End of file.**
