#!/usr/bin/env python3
import sys
import os
import numpy as np
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QPushButton, QFrame, QTabWidget, QSlider, 
                             QListWidget, QListWidgetItem, QMessageBox, QSplitter,
                             QComboBox, QDoubleSpinBox, QAbstractSpinBox, QRadioButton, QButtonGroup)
from PyQt5.QtGui import QVector3D, QMatrix4x4, QColor, QFont, QPixmap
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread
from tf2_ros import Buffer, TransformListener
import pyqtgraph.opengl as gl
from stl import mesh
from scipy.spatial.transform import Rotation as R

MESH_PATH = "/home/vboxuser/rk_ws/src/rk_demo/meshes" 
LOGO_PATH = "/home/vboxuser/rk_ws/logo.png" 

# 1. KİNEMATİK MOTORU
class KinematicsEngine:
    def __init__(self):
        self.chain = [
            {'name': '1_base.STL', 'type': 'fixed', 'xyz': [0,0,0], 'rpy': [0,0,0], 'axis': [0,0,0], 'visual_xyz': [0,0,0], 'visual_rpy': [0,0,0]},
            {'name': '2_rotary_joint.STL', 'type': 'revolute', 'xyz': [0.0, 0.0, 0.0], 'rpy': [0.0, 0.0, 0.0], 'axis': [0.0, 0.0, 1.0], 'visual_xyz': [0,0,0], 'visual_rpy': [0,0,0]},
            {'name': '4_lower_arm.STL', 'type': 'revolute', 'xyz': [0.06000, 0.26000, 0.64000], 'rpy': [-0.17453, -1.57080, 0.00000], 'axis': [0.0, 0.0, 1.0], 'visual_xyz': [0,0,0], 'visual_rpy': [0, 1.5708, 0]},
            {'name': '5_upper_arm.STL', 'type': 'revolute', 'xyz': [0.44000, 0.48717, -0.05626], 'rpy': [0.0, 0.0, 0.0], 'axis': [0.0, 0.0, 1.0], 'visual_xyz': [0,0,0], 'visual_rpy': [0, 1.5708, 0]},
            {'name': '6_wrist_support.STL', 'type': 'revolute', 'xyz': [-0.05000, 0.52078, 0.04106], 'rpy': [0.01066, 0.08661, 1.69343], 'axis': [0.99714, 0.0, 0.07554], 'visual_xyz': [0,0,0], 'visual_rpy': [3.14159, -1.5708, 1.5708]},
            {'name': '7_wrist.STL', 'type': 'revolute', 'xyz': [0.16404, 0.00001, 0.12645], 'rpy': [1.57080, 1.48353, -0.12217], 'axis': [0.99714, 0.0, 0.07554], 'visual_xyz': [0,0,0], 'visual_rpy': [0, 0, 0]}
        ]

    def compute_transform(self, params, theta):
        T_trans = np.eye(4); T_trans[:3, 3] = params['xyz']
        r_static = R.from_euler('xyz', params['rpy'])
        T_rot_static = np.eye(4); T_rot_static[:3, :3] = r_static.as_matrix()
        axis = np.array(params['axis'])
        if np.linalg.norm(axis) > 0 and params['type'] == 'revolute':
            r_dynamic = R.from_rotvec(axis * theta)
            T_rot_dynamic = np.eye(4); T_rot_dynamic[:3, :3] = r_dynamic.as_matrix()
        else: T_rot_dynamic = np.eye(4)
        return T_trans @ T_rot_static @ T_rot_dynamic

    def get_visual_transform(self, params):
        T_v_trans = np.eye(4); T_v_trans[:3, 3] = params.get('visual_xyz', [0,0,0])
        r_v = R.from_euler('xyz', params.get('visual_rpy', [0,0,0]))
        T_v_rot = np.eye(4); T_v_rot[:3, :3] = r_v.as_matrix()
        return T_v_trans @ T_v_rot

    def forward_kinematics(self, joints):
        transforms = []; T_total = np.eye(4)
        transforms.append(T_total @ self.get_visual_transform(self.chain[0])) 
        for i in range(1, len(self.chain)):
            T_total = np.dot(T_total, self.compute_transform(self.chain[i], joints[i-1]))
            transforms.append(np.dot(T_total, self.get_visual_transform(self.chain[i])))
        return transforms, T_total[:3, 3]

    def solve_ik(self, target_pos, current_joints):
        q = np.array(current_joints); alpha = 0.2; max_iters = 50; tolerance = 0.01 
        for _ in range(max_iters):
            _, current_ee_pos = self.forward_kinematics(q)
            error = target_pos - current_ee_pos
            if np.linalg.norm(error) < tolerance: return q
            J = np.zeros((3, 5)); epsilon = 1e-4
            for i in range(5):
                q_p = q.copy(); q_p[i] += epsilon; _, pos_p = self.forward_kinematics(q_p)
                J[:, i] = (pos_p - current_ee_pos) / epsilon
            q = q + alpha * np.dot(np.linalg.pinv(J), error)
            q = np.clip(q, -3.14, 3.14)
        return q

# 2. GÖRÜNTÜLEYİCİ

class GhostRobotVisualizer(gl.GLViewWidget):
    targetMoved = pyqtSignal(list)
    jointJogged = pyqtSignal(int, float)

    def __init__(self, engine):
        super().__init__()
        self.engine = engine
        self.setCameraPosition(distance=4.5, elevation=25, azimuth=-135)
        self.setBackgroundColor('#dcdcdc') 
        self.opts['ambient'] = (0.6, 0.6, 0.6, 1.0) 
        self.opts['lightPosition'] = QVector3D(10, 20, 30)

        g = gl.GLGridItem(); g.setSize(x=4, y=4); g.setSpacing(x=0.5, y=0.5)
        g.setColor((0, 0, 0, 60)) 
        self.addItem(g)
        self.addItem(gl.GLAxisItem(size=QVector3D(0.3,0.3,0.3)))

        # Target (Gizli)
        self.target_pos = np.array([0.8, 0.0, 0.8])
        
        self.trace_points = []
        self.trace_line = gl.GLLinePlotItem(pos=np.array([[0,0,0]]), color=(0, 0.2, 0.7, 1), width=2, antialias=True)
        self.addItem(self.trace_line)

        self.meshes = []
        self.default_colors = []
        self.active_joint_idx = -1 # -1: View Mode
        
        self.load_robot_meshes()
        self.prev_mouse_pos = None

    def load_stl(self, filename, color):
        path = os.path.join(MESH_PATH, filename)
        try:
            mesh_data = mesh.Mesh.from_file(path)
            points = mesh_data.vectors.reshape(-1, 3)
            faces = np.arange(points.shape[0]).reshape(-1, 3)
            return gl.GLMeshItem(vertexes=points, faces=faces, drawEdges=False, smooth=False, shader='shaded', color=color, glOptions='opaque')
        except: return gl.GLBoxItem(size=QVector3D(0.1, 0.1, 0.1), color=color)

    def load_robot_meshes(self):
        self.default_colors = [(0.3, 0.3, 0.3, 1.0)] + [(0.9, 0.5, 0.0, 1.0)] * 5
        for i, link in enumerate(self.engine.chain):
            m = self.load_stl(link['name'], self.default_colors[i])
            self.addItem(m); self.meshes.append(m)

    def set_active_joint(self, idx):
        self.active_joint_idx = idx
        for i, m in enumerate(self.meshes):
            target_idx = i - 1
            if idx != -1 and target_idx == idx:
                m.setColor((0.0, 0.9, 0.0, 1.0)) # PARLAK YEŞİL (SEÇİLİ)
            else:
                m.setColor(self.default_colors[i])

    def update_ghost(self, joints):
        transforms, ee_pos = self.engine.forward_kinematics(joints)
        for i, T in enumerate(transforms):
            self.meshes[i].setTransform(QMatrix4x4(*T.flatten()))
        
        if len(self.trace_points) == 0 or np.linalg.norm(self.trace_points[-1] - ee_pos) > 0.02:
            self.trace_points.append(ee_pos)
            if len(self.trace_points) > 200: self.trace_points.pop(0) 
            self.trace_line.setData(pos=np.array(self.trace_points))

    def mousePressEvent(self, ev):
        self.prev_mouse_pos = ev.pos()
        super().mousePressEvent(ev)

    def mouseMoveEvent(self, ev):
        if self.prev_mouse_pos is None: return
        diff = ev.pos() - self.prev_mouse_pos
        self.prev_mouse_pos = ev.pos()

        if ev.buttons() == Qt.LeftButton:
            if self.active_joint_idx >= 0:
                # JOG MODE
                delta = diff.x() * 0.01
                self.jointJogged.emit(self.active_joint_idx, delta)
                ev.accept() 
            else:
                # VIEW MODE
                super().mouseMoveEvent(ev)
        else:
            super().mouseMoveEvent(ev)


# 3. YÖRÜNGE OYNATICI

class PathPlayer(QThread):
    step_signal = pyqtSignal(list) 
    finished_signal = pyqtSignal()

    def __init__(self, targets_list, publisher, start_joints, move_duration=2.0):
        super().__init__()
        self.full_path = [start_joints] + targets_list
        self.pub = publisher
        self.move_duration = float(move_duration) 
        self.running = False

    def run(self):
        self.running = True
        msg = JointTrajectory()
        msg.joint_names = ['base_to_rotary', 'rotary_to_lower', 'lower_to_upper', 'upper_to_support', 'support_to_wrist']
        current_time = 0.0
        for joints in self.full_path[1:]:
            current_time += self.move_duration
            pt = JointTrajectoryPoint()
            pt.positions = joints
            sec = int(current_time); nanosec = int((current_time - sec) * 1e9)
            pt.time_from_start.sec = sec; pt.time_from_start.nanosec = nanosec
            msg.points.append(pt)
        self.pub.publish(msg)
        
        fps = 30; steps_per_move = int(self.move_duration * fps); dt = 1.0 / fps
        for i in range(len(self.full_path) - 1):
            start_pose = np.array(self.full_path[i]); end_pose = np.array(self.full_path[i+1])
            for step in range(steps_per_move):
                if not self.running: break
                alpha = (step + 1) / steps_per_move
                current_pose = start_pose + (end_pose - start_pose) * alpha
                self.step_signal.emit(current_pose.tolist())
                time.sleep(dt)
            time.sleep(0.05)
        self.finished_signal.emit()

    def stop(self): self.running = False

# 4. ANA UYGULAMA
class RobotStudioApp(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.engine = KinematicsEngine()
        self.ghost_joints = [0.0] * 5
        self.saved_targets = [] 
        
        self.traj_pub = self.node.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        self.init_ui()
        self.timer = QTimer(); self.timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0)); self.timer.start(20)

    def init_ui(self):
        self.setWindowTitle("ENRO Studio v1.0 🚀")
        self.setGeometry(100, 100, 1400, 800)
        
        self.setStyleSheet("""
            QWidget { background-color: #f0f0f0; color: #333; font-family: 'Segoe UI', sans-serif; }
            QSlider::handle:horizontal { background: #e67e22; width: 15px; border-radius: 5px; }
            QPushButton { background-color: #e0e0e0; border: 1px solid #bbb; padding: 4px; border-radius: 4px; font-weight: bold; color: #333; }
            QPushButton:hover { background-color: #d0d0d0; }
            QPushButton#ArrowBtn { font-size: 14px; padding: 0px; width: 25px; }
            QListWidget { background-color: #fff; border: 1px solid #ccc; color: #333; }
            QLabel { font-size: 12px; color: #333; }
            QComboBox { background-color: #fff; border: 1px solid #ccc; padding: 5px; border-radius: 4px; }
            QFrame { background-color: #e6e6e6; border-radius: 5px; }
            QDoubleSpinBox { background-color: white; border: 1px solid #aaa; padding: 4px; min-width: 70px; font-size: 13px; font-weight: bold; }
            QRadioButton { font-weight: bold; color: #444; }
            QRadioButton::indicator:checked { background-color: #e67e22; border: 2px solid #333; border-radius: 6px; }
        """)
        
        main_layout = QVBoxLayout()
        
        # --- ÜST BAR (LOGO + BAŞLIK) ---
        top_bar = QHBoxLayout()
        top_bar.setContentsMargins(10, 5, 10, 5)
        
        # LOGO
        logo_label = QLabel()
        if os.path.exists(LOGO_PATH):
            pixmap = QPixmap(LOGO_PATH)
            pixmap = pixmap.scaledToHeight(50, Qt.SmoothTransformation) 
            logo_label.setPixmap(pixmap)
        else:
            logo_label.setText("LOGO")
            logo_label.setStyleSheet("font-weight: bold; color: #555; background: #ddd; padding: 5px;")
        
        top_bar.addWidget(logo_label)
        
        # BAŞLIK
        lbl_header = QLabel("  ENRO STUDIO")
        lbl_header.setStyleSheet("font-size: 20px; font-weight: 900; color: #333; letter-spacing: 1px;")
        top_bar.addWidget(lbl_header)
        
        top_bar.addStretch()
        main_layout.addLayout(top_bar)
        
        # --- ORTA ALAN ---
        center_layout = QHBoxLayout()
        
        self.view_3d = GhostRobotVisualizer(self.engine)
        self.view_3d.targetMoved.connect(self.on_ik_target_move)
        self.view_3d.jointJogged.connect(self.on_joint_jog)
        center_layout.addWidget(self.view_3d, stretch=3)
        
        right_panel = QVBoxLayout()
        right_panel.setContentsMargins(10, 0, 10, 0)
        
        # Kontrol Başlığı
        lbl_panel_title = QLabel("ENRO STUDIO")
        lbl_panel_title.setStyleSheet("font-size: 16px; font-weight: bold; color: #e67e22; margin-bottom: 5px;")
        lbl_panel_title.setAlignment(Qt.AlignCenter)
        right_panel.addWidget(lbl_panel_title)

        lbl_hint = QLabel("Select Joint -> Drag Mouse Left/Right")
        lbl_hint.setStyleSheet("color: #666; font-style: italic; font-size: 11px;")
        lbl_hint.setAlignment(Qt.AlignCenter)
        right_panel.addWidget(lbl_hint)

        self.lbl_pos = QLabel("TCP: X:0.00 Y:0.00 Z:0.00")
        self.lbl_pos.setStyleSheet("font-family: monospace; font-size: 13px; background: #fff; padding: 4px; border: 1px solid #ccc; border-radius: 4px;")
        right_panel.addWidget(self.lbl_pos)
        
        grp_joints = QFrame()
        l_joints = QVBoxLayout(grp_joints)
        
        self.sliders = []
        self.spinboxes = []
        self.btn_grp = QButtonGroup()
        
        # --- VIEW MODE BUTONU ---
        h_view = QHBoxLayout()
        rb_view = QRadioButton("👁️ VIEW MODE")
        rb_view.setChecked(True) 
        self.btn_grp.addButton(rb_view, -1)
        self.btn_grp.idClicked.connect(self.on_radio_select)
        h_view.addWidget(rb_view)
        h_view.addStretch()
        l_joints.addLayout(h_view)

        # --- JOINT SEÇİMLERİ ---
        for i in range(5):
            h = QHBoxLayout()
            rb = QRadioButton(f"J{i+1}")
            self.btn_grp.addButton(rb, i)
            h.addWidget(rb)
            
            s = QSlider(Qt.Horizontal)
            s.setRange(-314, 314); s.setValue(0)
            s.valueChanged.connect(lambda val, idx=i: self.on_slider_change(idx))
            self.sliders.append(s)
            h.addWidget(s)
            
            btn_down = QPushButton("▼"); btn_down.setObjectName("ArrowBtn"); btn_down.setFixedSize(25, 25)
            sb = QDoubleSpinBox(); sb.setRange(-3.14, 3.14); sb.setSingleStep(0.01); sb.setDecimals(2)
            sb.setButtonSymbols(QAbstractSpinBox.NoButtons); sb.setAlignment(Qt.AlignCenter)
            sb.valueChanged.connect(lambda val, idx=i: self.on_spinbox_change(val, idx))
            btn_up = QPushButton("▲"); btn_up.setObjectName("ArrowBtn"); btn_up.setFixedSize(25, 25)
            
            btn_down.clicked.connect(lambda _, sb=sb: sb.setValue(sb.value() - 0.01))
            btn_up.clicked.connect(lambda _, sb=sb: sb.setValue(sb.value() + 0.01))
            
            h.addWidget(btn_down); h.addWidget(sb); h.addWidget(btn_up)
            self.spinboxes.append(sb)
            l_joints.addLayout(h)
            
        right_panel.addWidget(grp_joints)

        right_panel.addWidget(QLabel("📍 TARGETS & PATH"))
        self.list_targets = QListWidget()
        right_panel.addWidget(self.list_targets)

        h_btns = QHBoxLayout()
        btn_teach = QPushButton("📌 TEACH")
        btn_teach.clicked.connect(self.teach_target)
        h_btns.addWidget(btn_teach)

        btn_clear = QPushButton("🗑️ CLEAR")
        btn_clear.clicked.connect(self.clear_targets)
        h_btns.addWidget(btn_clear)
        right_panel.addLayout(h_btns)

        right_panel.addStretch()
        
        play_layout = QHBoxLayout()
        self.combo_speed = QComboBox()
        self.combo_speed.addItems(["🐢 Slow", "🚶 Normal", "🐇 Fast", "⚡ Turbo"])
        self.combo_speed.setCurrentIndex(1) 
        play_layout.addWidget(self.combo_speed)

        btn_play = QPushButton("▶️ PLAY PATH")
        btn_play.setStyleSheet("background-color: #2980b9; color: white; padding: 10px; border: none;")
        btn_play.clicked.connect(self.play_path)
        play_layout.addWidget(btn_play)
        
        right_panel.addLayout(play_layout)

        btn_move = QPushButton("🚀 MOVE TO CURRENT")
        btn_move.setStyleSheet("background-color: #27ae60; color: white; padding: 10px; border: none;")
        btn_move.clicked.connect(self.move_real_robot)
        right_panel.addWidget(btn_move)

        btn_home = QPushButton("🏠 HOME")
        btn_home.clicked.connect(self.go_home)
        right_panel.addWidget(btn_home)
        
        center_layout.addLayout(right_panel, stretch=1)
        main_layout.addLayout(center_layout)
        
        self.setLayout(main_layout)
        self.on_radio_select(-1)
        self.view_3d.update_ghost(self.ghost_joints)

    def on_radio_select(self, idx):
        self.view_3d.set_active_joint(idx)

    def on_joint_jog(self, idx, delta):
        new_val = self.spinboxes[idx].value() + delta
        new_val = max(-3.14, min(3.14, new_val))
        self.spinboxes[idx].setValue(new_val)

    def on_ik_target_move(self, pos):
        self.lbl_pos.setText(f"TCP: X:{pos[0]:.2f} Y:{pos[1]:.2f} Z:{pos[2]:.2f}")

    def on_slider_change(self, idx):
        val = self.sliders[idx].value() / 100.0
        self.ghost_joints[idx] = val
        self.spinboxes[idx].blockSignals(True)
        self.spinboxes[idx].setValue(val)
        self.spinboxes[idx].blockSignals(False)
        self.update_ghost_visual()

    def on_spinbox_change(self, val, idx):
        self.ghost_joints[idx] = val
        self.sliders[idx].blockSignals(True)
        self.sliders[idx].setValue(int(val * 100))
        self.sliders[idx].blockSignals(False)
        self.update_ghost_visual()

    def update_ghost_visual(self):
        _, ee_pos = self.engine.forward_kinematics(self.ghost_joints)
        self.view_3d.target_pos = ee_pos
        self.view_3d.update_ghost(self.ghost_joints)
        self.lbl_pos.setText(f"TCP: X:{ee_pos[0]:.2f} Y:{ee_pos[1]:.2f} Z:{ee_pos[2]:.2f}")

    def update_ui_sync(self):
        self.view_3d.update_ghost(self.ghost_joints)
        for i, val in enumerate(self.ghost_joints):
            self.sliders[i].blockSignals(True)
            self.sliders[i].setValue(int(val * 100))
            self.sliders[i].blockSignals(False)
            self.spinboxes[i].blockSignals(True)
            self.spinboxes[i].setValue(val)
            self.spinboxes[i].blockSignals(False)

    def teach_target(self):
        idx = len(self.saved_targets) + 10
        name = f"Target_{idx}"
        joints = list(self.ghost_joints) 
        self.saved_targets.append(joints)
        item = QListWidgetItem(f"{name}  [J1: {joints[0]:.2f} ...]")
        item.setForeground(QColor("#d35400"))
        self.list_targets.addItem(item)

    def clear_targets(self):
        self.saved_targets = []
        self.list_targets.clear()
        self.view_3d.trace_points = []
        self.view_3d.trace_line.setData(pos=np.array([[0,0,0]]))

    def play_path(self):
        if not self.saved_targets:
            QMessageBox.warning(self, "Hata", "Önce birkaç nokta öğretmelisin (Teach Target)!")
            return
        idx = self.combo_speed.currentIndex()
        durations = [4.0, 2.0, 1.0, 0.5]
        selected_duration = durations[idx]
        self.player = PathPlayer(self.saved_targets, self.traj_pub, self.ghost_joints, selected_duration)
        self.player.step_signal.connect(self.sync_ghost_from_player)
        self.player.start()

    def sync_ghost_from_player(self, joints):
        self.ghost_joints = joints
        self.update_ui_sync()

    def move_real_robot(self):
        msg = JointTrajectory()
        msg.joint_names = ['base_to_rotary', 'rotary_to_lower', 'lower_to_upper', 'upper_to_support', 'support_to_wrist']
        pt = JointTrajectoryPoint()
        pt.positions = self.ghost_joints
        pt.time_from_start.sec = 2 
        msg.points.append(pt)
        self.traj_pub.publish(msg)

    def go_home(self):
        self.ghost_joints = [0.0] * 5
        self.update_ui_sync()
        self.move_real_robot()

def main(args=None):
    rclpy.init(args=args)
    node = Node('robot_studio_gui')
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = RobotStudioApp(node)
    window.show()
    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
