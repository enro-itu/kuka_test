#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys

from PyQt5 import QtWidgets, QtCore

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

# ROS
class ArmUiNode(Node):
    def __init__(self, ui_ref):
        super().__init__("rk_arm_ui")
        self.ui = ui_ref

        # Publisher
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            "/joint_group_position_controller/commands",
            10
        )

        # Subscriber
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg: JointState):
        desired_joints = ["base_to_rotary", "rotary_to_lower", "lower_to_upper"]

        try:
            idx1 = msg.name.index(desired_joints[0])
            idx2 = msg.name.index(desired_joints[1])
            idx3 = msg.name.index(desired_joints[2])
        except ValueError:
            return

        j1 = float(msg.position[idx1])
        j2 = float(msg.position[idx2])
        j3 = float(msg.position[idx3])

        self.ui.joint_state_signal.emit(j1, j2, j3)

    def send_joint_command(self, j1: float, j2: float, j3: float):
        msg = Float64MultiArray()
        msg.data = [j1, j2, j3]
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Sent command: {msg.data}")


# QT
class ArmControlWindow(QtWidgets.QMainWindow):
    joint_state_signal = QtCore.pyqtSignal(float, float, float)

    def __init__(self):
        super().__init__()

        self.setWindowTitle("rk_demo Arm UI")
        self.resize(600, 400)

        self.node = None

        # main layout
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        central_widget.setStyleSheet("background-color: #BFE9FF;")
        main_layout = QtWidgets.QVBoxLayout()
        central_widget.setLayout(main_layout)

        # JOINT CONTROL GROUP
        joint_group = QtWidgets.QGroupBox("Joint Controls")
        joint_layout = QtWidgets.QGridLayout()
        joint_group.setLayout(joint_layout)

        slider_min, slider_max = -180, 180

        # JOINT 1
        lbl_j1 = QtWidgets.QLabel("Joint 1")
        self.slider_j1 = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_j1.setRange(slider_min, slider_max)

        self.spin_j1 = QtWidgets.QDoubleSpinBox()
        self.spin_j1.setRange(slider_min, slider_max)
        self.spin_j1.setDecimals(1)

        joint_layout.addWidget(lbl_j1, 0, 0)
        joint_layout.addWidget(self.slider_j1, 0, 1)
        joint_layout.addWidget(self.spin_j1, 0, 2)

        # JOINT 2
        lbl_j2 = QtWidgets.QLabel("Joint 2")
        self.slider_j2 = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_j2.setRange(slider_min, slider_max)

        self.spin_j2 = QtWidgets.QDoubleSpinBox()
        self.spin_j2.setRange(slider_min, slider_max)
        self.spin_j2.setDecimals(1)

        joint_layout.addWidget(lbl_j2, 1, 0)
        joint_layout.addWidget(self.slider_j2, 1, 1)
        joint_layout.addWidget(self.spin_j2, 1, 2)

        # JOINT 3
        lbl_j3 = QtWidgets.QLabel("Joint 3")
        self.slider_j3 = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_j3.setRange(slider_min, slider_max)

        self.spin_j3 = QtWidgets.QDoubleSpinBox()
        self.spin_j3.setRange(slider_min, slider_max)
        self.spin_j3.setDecimals(1)

        joint_layout.addWidget(lbl_j3, 2, 0)
        joint_layout.addWidget(self.slider_j3, 2, 1)
        joint_layout.addWidget(self.spin_j3, 2, 2)

        main_layout.addWidget(joint_group)

        # BUTTONS
        btn_layout = QtWidgets.QHBoxLayout()
        self.btn_home = QtWidgets.QPushButton("Home")
        self.btn_reset = QtWidgets.QPushButton("Reset UI")
        self.btn_send = QtWidgets.QPushButton("Send")
        btn_style = """
        QPushButton {
            background-color: #F6E58D;
            color: #2d3436;
            border: 1px solid #dcdde1;
            border-radius: 8px;
            padding: 8px 12px;
            font-weight: 600;
        }
        QPushButton:hover {
            background-color: #F3D56B;
        }
        QPushButton:pressed {
            background-color: #E9C654;
        }
        QPushButton:disabled {
            background-color: #f1f2f6;
            color: #7f8c8d;
        }
        """
        self.btn_home.setStyleSheet(btn_style)
        self.btn_reset.setStyleSheet(btn_style)
        self.btn_send.setStyleSheet(btn_style)

        btn_layout.addStretch()
        btn_layout.addWidget(self.btn_home)
        btn_layout.addWidget(self.btn_reset)
        btn_layout.addWidget(self.btn_send)
        main_layout.addLayout(btn_layout)

        # STATUS AREA
        self.status = QtWidgets.QTextEdit()
        self.status.setReadOnly(True)
        self.status.setPlaceholderText("Status messages will appear here...")
        self.status.setStyleSheet("background-color: white; border-radius: 6px;")
        main_layout.addWidget(self.status)

        # Slider ↔️ Spin sync
        self.slider_j1.valueChanged.connect(self.sync_slider_to_spin)
        self.slider_j2.valueChanged.connect(self.sync_slider_to_spin)
        self.slider_j3.valueChanged.connect(self.sync_slider_to_spin)

        self.spin_j1.valueChanged.connect(self.sync_spin_to_slider)
        self.spin_j2.valueChanged.connect(self.sync_spin_to_slider)
        self.spin_j3.valueChanged.connect(self.sync_spin_to_slider)

        # Buttons
        self.btn_send.clicked.connect(self.on_send_clicked)
        self.btn_home.clicked.connect(self.on_home_clicked)
        self.btn_reset.clicked.connect(self.on_reset_clicked)

        self.joint_state_signal.connect(self.on_joint_state_update)

        self.set_home_pose()

    # DIŞARIDAN NODE BAĞLAMA
    def set_node(self, node: ArmUiNode):
        """ROS node referansını UI'ye ver."""
        self.node = node


    # SYNC FUNCTIONS
    def sync_slider_to_spin(self):
        """Slider değişince spinbox'ları güncelle."""
        self.spin_j1.blockSignals(True)
        self.spin_j2.blockSignals(True)
        self.spin_j3.blockSignals(True)

        self.spin_j1.setValue(self.slider_j1.value())
        self.spin_j2.setValue(self.slider_j2.value())
        self.spin_j3.setValue(self.slider_j3.value())

        self.spin_j1.blockSignals(False)
        self.spin_j2.blockSignals(False)
        self.spin_j3.blockSignals(False)

    def sync_spin_to_slider(self):
        """Spinbox değişince slider'ları güncelle."""
        self.slider_j1.blockSignals(True)
        self.slider_j2.blockSignals(True)
        self.slider_j3.blockSignals(True)

        self.slider_j1.setValue(int(self.spin_j1.value()))
        self.slider_j2.setValue(int(self.spin_j2.value()))
        self.slider_j3.setValue(int(self.spin_j3.value()))

        self.slider_j1.blockSignals(False)
        self.slider_j2.blockSignals(False)
        self.slider_j3.blockSignals(False)

    def set_home_pose(self):
        """UI'de home pozisyonunu (0,0,0) olarak set et."""
        self.spin_j1.setValue(0.0)
        self.spin_j2.setValue(0.0)
        self.spin_j3.setValue(0.0)
        self.status.append("Home pose set.")

    # BUTTON SLOTS
    def on_send_clicked(self):
        """Send butonuna basıldığında eklem değerlerini node'a gönder."""
        if self.node is None:
            self.status.append("Node not connected, cannot send command.")
            return

        j1 = float(self.spin_j1.value())
        j2 = float(self.spin_j2.value())
        j3 = float(self.spin_j3.value())

        self.node.send_joint_command(j1, j2, j3)
        self.status.append(f"Sent command: [{j1:.2f}, {j2:.2f}, {j3:.2f}]")

    def on_home_clicked(self):
        """Home pozisyonunu uygula (0,0,0)."""
        self.spin_j1.setValue(0.0)
        self.spin_j2.setValue(0.0)
        self.spin_j3.setValue(0.0)
        self.status.append("Home pose applied.")

    def on_reset_clicked(self):
        """UI bileşenlerini home durumuna döndürür."""
        self.spin_j1.setValue(0.0)
        self.spin_j2.setValue(0.0)
        self.spin_j3.setValue(0.0)
        self.status.append("UI reset to home pose.")

    # JOINT STATE SLOT 
    def on_joint_state_update(self, j1: float, j2: float, j3: float):
        """ROS'tan gelen eklem açılarını UI'ye uygular."""
        self.spin_j1.blockSignals(True)
        self.spin_j2.blockSignals(True)
        self.spin_j3.blockSignals(True)

        self.spin_j1.setValue(j1)
        self.spin_j2.setValue(j2)
        self.spin_j3.setValue(j3)

        self.spin_j1.blockSignals(False)
        self.spin_j2.blockSignals(False)
        self.spin_j3.blockSignals(False)

        self.sync_spin_to_slider()
        self.status.append(f"Joint state update: [{j1:.2f}, {j2:.2f}, {j3:.2f}]")


# MAIN

def main():
    rclpy.init()

    app = QtWidgets.QApplication(sys.argv)

    ui = ArmControlWindow()
    ui.show()

    node = ArmUiNode(ui)
    ui.set_node(node)

    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    app.exec_()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
