# ASSIGNMENT — UI & Control Connections for rk_demo Robot Arm
**Project:** ENRO / rk_demo  
**Assigned to:** Avni  
**Reviewed by:** Emre / Software Team  
**Date:** 09.12.2025

---

# 1. Objective

Design and implement a **graphical User Interface (UI)** using **Qt (Option A: PyQt5 or PySide2)** to control the 3-DOF robot arm in the `rk_demo` package.

The UI must:
- Control joints in real time via ROS 2
- Show current joint states
- Provide preset pose buttons
- Be robust enough to work both with RViz/Gazebo or headless

The UI is a replacement for using `joint_state_publisher_gui` directly.

---

# 2. Deliverables

### Mandatory
- `rk_demo/ui_control/arm_ui.py` — main PyQt-based UI + ROS 2 node
- `rk_demo/ui_control/arm_ui.launch.py` — launch file to start the UI
- `rk_demo/docs/ui_user_guide.md` — short user manual
- Working connection to:
  - `/joint_group_position_controller/commands` (publisher)
  - `/joint_states` (subscriber)

### Optional (Bonus)
- Cartesian control mini-panel (X, Y, Z → IK → joints)
- Pose save/load to JSON
- Smooth motion / interpolation
- RViz dockable panel version of the UI

---

# 3. Target Directory Structure

```text
rk_demo/
 ├── ui_control/
 │    ├── __init__.py           (optional)
 │    ├── arm_ui.py             # main PyQt UI + ROS2 node
 │    └── arm_ui.launch.py      # launch file
 ├── docs/
 │    └── ui_user_guide.md
 └── arif.md
 └── avni.md   # this assignment file
```

If any folders don’t exist, create them.

---

# 4. UI Design Requirements

## 4.1 General

Use **Qt** via **PyQt5** or **PySide2**:

- Single main window with:
  - Sliders for `joint1`, `joint2`, `joint3`
  - Numeric fields (SpinBoxes) next to each slider
  - Buttons:
    - **Send** (send current UI joint values)
    - **Home** (go to predefined home pose)
    - **Reset UI** (reset sliders/fields to home or zero)
  - A status text area / label for logs and messages
  - Live display of current joint values from `/joint_states`

## 4.2 ROS 2 Connections

### Publisher
- Topic: `/joint_group_position_controller/commands`
- Type: `std_msgs/msg/Float64MultiArray`
- Content:
  ```python
  msg.data = [j1, j2, j3]
  ```

### Subscriber
- Topic: `/joint_states`
- Type: `sensor_msgs/msg/JointState`
- Use to:
  - Update the sliders & spinboxes
  - Show real-time joint values

### Node

- Node name: `rk_arm_ui`
- Use `rclpy` inside `arm_ui.py`
- Handle ROS in a non-blocking way (e.g., QTimer → spin some work, or threaded executor)

---

# 5. Implementation Tasks

## Task 1 — Basic PyQt Window Layout

### Goal
Create the basic UI layout **without** ROS.

### Steps
1. Create `arm_ui.py` with:
   - `class ArmControlWindow(QMainWindow):`
   - Three sliders `slider_j1`, `slider_j2`, `slider_j3`
   - Three spinboxes `spin_j1`, `spin_j2`, `spin_j3`
   - Buttons: `btn_send`, `btn_home`, `btn_reset`
   - A status label or QTextEdit (read-only)

2. Link sliders & spinboxes:
   - Moving a slider updates its spinbox
   - Changing a spinbox updates its slider

### Template (simplified)

```python
from PyQt5 import QtWidgets, QtCore

class ArmControlWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("rk_demo Arm UI")

        # TODO: create widgets
        # self.slider_j1 = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        # self.spin_j1 = QtWidgets.QDoubleSpinBox()
        # ...

        # TODO: layout with QVBoxLayout / QGridLayout

        # TODO: connect signals
        # self.slider_j1.valueChanged.connect(self.on_slider_j1_changed)
        # self.spin_j1.valueChanged.connect(self.on_spin_j1_changed)

    # TODO: implement slots
    # def on_slider_j1_changed(self, value):
    #     ...

def main():
    app = QtWidgets.QApplication([])
    win = ArmControlWindow()
    win.show()
    app.exec_()
```

### TODO for Avni
- [ ] Build the PyQt layout with all widgets
- [ ] Ensure sliders & spinboxes stay in sync
- [ ] Add basic styling (labels, group boxes) to make it clean

---

## Task 2 — Integrate ROS 2 (rclpy) with the UI

### Goal
Run a ROS 2 node in the same process as the PyQt app.

### Steps
1. Initialize `rclpy` in `main()` or inside a helper class
2. Create a node, e.g. `ArmUiNode(rclpy.node.Node)`
3. Connect ROS callbacks to update the UI
4. Use QTimer or a background thread to periodically call `rclpy.spin_once(node, timeout_sec=0.01)`

### Template (conceptual)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from PyQt5 import QtWidgets, QtCore

class ArmUiNode(Node):
    def __init__(self, ui):
        super().__init__("rk_arm_ui")
        self.ui = ui
        self.cmd_pub = self.create_publisher(Float64MultiArray,
                                             "/joint_group_position_controller/commands",
                                             10)
        self.create_subscription(JointState,
                                 "/joint_states",
                                 self.joint_state_cb,
                                 10)

    def joint_state_cb(self, msg: JointState):
        # TODO: call into UI thread safely
        # self.ui.update_joint_states_from_ros(msg)

class ArmControlWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        # TODO: setup UI

    def update_joint_states_from_ros(self, msg):
        # TODO: move to main thread using signals/slots if needed
        pass

def main():
    rclpy.init()
    app = QtWidgets.QApplication([])

    ui = ArmControlWindow()
    node = ArmUiNode(ui)

    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    ui.show()
    app.exec_()

    node.destroy_node()
    rclpy.shutdown()
```

### TODO for Avni
- [ ] Create ROS node class
- [ ] Wire up publisher & subscriber
- [ ] Ensure UI is updated from ROS callbacks without freezing

---

## Task 3 — Sending Commands from the UI

### Goal
Pressing **Send** publishes the current joint values.

### Steps
1. Add a method to ArmControlWindow to read:
   - `spin_j1.value()`, `spin_j2.value()`, `spin_j3.value()`
2. Call a node method like `node.send_joint_command(j1, j2, j3)`
3. Node constructs `Float64MultiArray` and publishes.

### Template

```python
def on_send_clicked(self):
    j1 = self.spin_j1.value()
    j2 = self.spin_j2.value()
    j3 = self.spin_j3.value()
    self.node.send_joint_command(j1, j2, j3)

class ArmUiNode(Node):
    def send_joint_command(self, j1, j2, j3):
        msg = Float64MultiArray()
        msg.data = [j1, j2, j3]
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Sent command: {msg.data}")
```

### TODO for Avni
- [ ] Hook `btn_send.clicked` to publishing
- [ ] Confirm commands affect the robot in Gazebo / RViz

---

## Task 4 — Home & Reset Buttons

### Goal
Add quick pose operations.

### Definitions (can be tuned with Emre):
- **Home Pose:** `[0.0, 0.0, 0.0]`
- **Reset UI:** reset widgets to home pose values

### Steps
1. Implement `on_home_clicked`:
   - Set spinboxes & sliders to home values
   - Optionally auto-send the pose
2. Implement `on_reset_clicked`:
   - Just reset UI, no publish (or ask Emre)

### TODO for Avni
- [ ] Implement Home button logic
- [ ] Implement Reset UI logic
- [ ] Reflect final behavior in `ui_user_guide.md`

---

## Task 5 — Live Joint State Feedback

### Goal
Show real robot joint values in the UI.

### Steps
1. In `joint_state_cb`, read correct joint names & positions
2. Map to j1, j2, j3
3. Emit a Qt signal to update UI in main thread (important)

### Template

```python
class ArmControlWindow(QtWidgets.QMainWindow):
    joint_state_signal = QtCore.pyqtSignal(float, float, float)

    def __init__(self):
        super().__init__()
        self.joint_state_signal.connect(self.on_joint_state_update)

    def on_joint_state_update(self, j1, j2, j3):
        # Update sliders & spinboxes but avoid infinite loops
        # Option: blockSignals(True) around setValue()
        pass
```

In node:

```python
def joint_state_cb(self, msg):
    # TODO: extract j1, j2, j3 from msg
    self.ui.joint_state_signal.emit(j1, j2, j3)
```

### TODO for Avni
- [ ] Implement joint_state signal/slot mechanism
- [ ] Update UI gracefully

---

## Task 6 — Launch File

Create `arm_ui.launch.py` inside `ui_control/`:

- Launches the Python UI node using `ros2 run`-style `Node` action (executable is `arm_ui.py` entrypoint).
- You may add entry point in `setup.py` if required (`console_scripts`).

### Minimal Launch Template

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rk_demo",
            executable="arm_ui",
            name="rk_arm_ui",
            output="screen"
        )
    ])
```

### TODO for Avni
- [ ] Add console_scripts entry for `arm_ui` in `setup.py`
- [ ] Implement proper launch file

---

# 6. Documentation — ui_user_guide.md

Create `docs/ui_user_guide.md` containing:

- How to start the UI:
  ```bash
  ros2 launch rk_demo arm_ui.launch.py
  ```
- Requirements (PyQt5/PySide2 installed, rk_demo built)
- Basic usage:
  - Moving sliders
  - Sending commands
  - Using Home / Reset
- Notes:
  - Works with Gazebo or only with `robot_state_publisher`
  - Safety: avoid extreme positions if needed
- Troubleshooting:
  - UI opens but doesn’t move robot
  - No joint states appearing
  - ROS domain ID mismatch, etc.

### TODO for Avni
- [ ] Write concise, clear guide
- [ ] Include at least one screenshot of the UI (optional but good)

---

# 7. Acceptance Criteria

To mark this assignment **DONE**, the following must be true:

- [ ] `arm_ui.py` runs without crashes
- [ ] Launch via `ros2 launch rk_demo arm_ui.launch.py`
- [ ] Sliders & spinboxes are synced
- [ ] **Send** publishes to `/joint_group_position_controller/commands`
- [ ] Robot actually moves when connected to controller
- [ ] Joint states from `/joint_states` appear correctly in the UI
- [ ] Home & Reset buttons behave as defined
- [ ] Code is reasonably clean, commented, and readable
- [ ] `ui_user_guide.md` explains how to use the UI
- [ ] All changes are in a branch like:
  - `feature/ui-control-avni`
- [ ] PR opened and passes basic review

---

# 8. Notes for Avni

- Focus on **stability first**, then extra features.
- Keep ROS–UI interaction responsive (no blocking spin).
- If you get stuck on Qt + ROS integration:
  - Start with pure PyQt UI
  - Add ROS afterward in incremental steps
- Ask Emre for:
  - Joint naming conventions
  - Expected home pose
  - Any safety limits

---

**End of file.**
