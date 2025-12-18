#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from PyQt5 import QtWidgets, QtCore

class ArmUiNode(Node):
    def __init__(self, ui_signal):
        super().__init__("rk_arm_ui")
        self.ui_signal = ui_signal
        self.joint_names = ["base_to_rotary", "rotary_to_lower", "lower_to_upper"]
        
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            "/joint_group_position_controller/commands",
            10
        )
        
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_cb,
            10
        )

    def send_joint_command(self, j1, j2, j3):
        msg = Float64MultiArray()
        msg.data = [float(j1), float(j2), float(j3)]
        self.cmd_pub.publish(msg)

    def joint_state_cb(self, msg: JointState):
        try:
            if self.joint_names[0] in msg.name:
                idx1 = msg.name.index(self.joint_names[0])
                idx2 = msg.name.index(self.joint_names[1])
                idx3 = msg.name.index(self.joint_names[2])

                val1 = msg.position[idx1]
                val2 = msg.position[idx2]
                val3 = msg.position[idx3]

                self.ui_signal.emit(val1, val2, val3)
        except (ValueError, IndexError):
            pass

class ArmControlWindow(QtWidgets.QMainWindow):
    joint_state_signal = QtCore.pyqtSignal(float, float, float)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("rk_demo Robot Arm UI")
        self.resize(500, 400)
        
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        self.main_layout = QtWidgets.QVBoxLayout(central_widget)

        self.create_header()
        
        self.slider_j1, self.spin_j1 = self.create_joint_control("Joint 1 (Base)", -3.14, 3.14)
        self.slider_j2, self.spin_j2 = self.create_joint_control("Joint 2 (Shoulder)", -3.14, 3.14)
        self.slider_j3, self.spin_j3 = self.create_joint_control("Joint 3 (Elbow)", -3.14, 3.14)

        self.create_buttons()

        self.status_label = QtWidgets.QLabel("Status: Ready")
        self.status_label.setAlignment(QtCore.Qt.AlignCenter)
        self.main_layout.addWidget(self.status_label)

        self.node = None
        self.joint_state_signal.connect(self.update_joint_states_from_ros)

    def create_header(self):
        title = QtWidgets.QLabel("RK_DEMO 3-DOF ARM CONTROL")
        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setStyleSheet("font-weight: bold; font-size: 16px; margin-bottom: 10px;")
        self.main_layout.addWidget(title)

    def create_joint_control(self, label_text, min_val, max_val):
        group = QtWidgets.QGroupBox(label_text)
        layout = QtWidgets.QHBoxLayout()

        slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        slider.setRange(int(min_val * 100), int(max_val * 100))
        slider.setValue(0)

        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(min_val, max_val)
        spin.setSingleStep(0.1)
        spin.setValue(0.0)
        spin.setDecimals(2)

        slider.valueChanged.connect(lambda val: spin.setValue(val / 100.0))
        spin.valueChanged.connect(lambda val: slider.setValue(int(val * 100.0)))

        layout.addWidget(slider)
        layout.addWidget(spin)
        group.setLayout(layout)
        self.main_layout.addWidget(group)

        return slider, spin

    def create_buttons(self):
        btn_layout = QtWidgets.QHBoxLayout()

        self.btn_home = QtWidgets.QPushButton("HOME")
        self.btn_home.clicked.connect(self.on_home_clicked)

        self.btn_reset = QtWidgets.QPushButton("RESET UI")
        self.btn_reset.clicked.connect(self.on_reset_clicked)

        self.btn_send = QtWidgets.QPushButton("SEND COMMAND")
        self.btn_send.clicked.connect(self.on_send_clicked)
        self.btn_send.setStyleSheet("font-weight: bold;")

        btn_layout.addWidget(self.btn_home)
        btn_layout.addWidget(self.btn_reset)
        btn_layout.addWidget(self.btn_send)
        self.main_layout.addLayout(btn_layout)

    def on_send_clicked(self):
        if self.node:
            j1 = self.spin_j1.value()
            j2 = self.spin_j2.value()
            j3 = self.spin_j3.value()
            self.node.send_joint_command(j1, j2, j3)
            self.status_label.setText(f"Sent: [{j1:.2f}, {j2:.2f}, {j3:.2f}]")

    def on_home_clicked(self):
        self.spin_j1.setValue(0.0)
        self.spin_j2.setValue(0.0)
        self.spin_j3.setValue(0.0)
        self.on_send_clicked()

    def on_reset_clicked(self):
        self.spin_j1.setValue(0.0)
        self.spin_j2.setValue(0.0)
        self.spin_j3.setValue(0.0)
        self.status_label.setText("UI Reset")

    def update_joint_states_from_ros(self, j1, j2, j3):
        widgets = [
            self.slider_j1, self.spin_j1,
            self.slider_j2, self.spin_j2,
            self.slider_j3, self.spin_j3
        ]
        
        for w in widgets: w.blockSignals(True)

        self.spin_j1.setValue(j1)
        self.slider_j1.setValue(int(j1 * 100))
        
        self.spin_j2.setValue(j2)
        self.slider_j2.setValue(int(j2 * 100))
        
        self.spin_j3.setValue(j3)
        self.slider_j3.setValue(int(j3 * 100))

        for w in widgets: w.blockSignals(False)

def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)

    ui = ArmControlWindow()
    ros_node = ArmUiNode(ui.joint_state_signal)
    ui.node = ros_node

    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    timer.start(10)

    ui.show()
    
    try:
        sys.exit(app.exec_())
    except Exception:
        pass
    finally:
        timer.stop()
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
