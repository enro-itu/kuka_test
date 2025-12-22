#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QSlider, QPushButton
from PyQt5.QtCore import Qt

class RobotArmUI(QWidget):
    def __init__(self):
        super().__init__()
        
        # --- ROS 2 KURULUMU ---
        rclpy.init(args=None)
        self.node = Node('custom_ui_node')
        
        # Robotun dinlediği Topic (Doğru kontrolcü ismiyle)
        self.publisher = self.node.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # SENİN ROBOTUNUN GERÇEK EKLEM İSİMLERİ
        self.joint_names = [
            'base_to_rotary',   # 1. Eklem
            'rotary_to_lower',  # 2. Eklem
            'lower_to_upper'    # 3. Eklem
        ]
        
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("KUKA Robot Kontrol Paneli 🚀")
        self.setGeometry(100, 100, 400, 300)
        layout = QVBoxLayout()

        self.sliders = []
        self.labels = []

        # 3 Eklem için Slider oluştur
        joint_titles = ["Base (Dönüş)", "Shoulder (Omuz)", "Elbow (Dirsek)"]
        
        for i, name in enumerate(joint_titles):
            # Etiket
            lbl = QLabel(f"{name}: 0.00 rad")
            lbl.setAlignment(Qt.AlignCenter)
            layout.addWidget(lbl)
            self.labels.append(lbl)

            # Slider
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-314) # -3.14 radyan
            slider.setMaximum(314)  # +3.14 radyan
            slider.setValue(0)
            slider.valueChanged.connect(self.update_labels)
            layout.addWidget(slider)
            self.sliders.append(slider)

        # Gönder Butonu
        btn = QPushButton("HAREKET ETTİR (SEND)")
        btn.clicked.connect(self.send_command)
        btn.setStyleSheet("background-color: green; color: white; font-weight: bold; padding: 10px;")
        layout.addWidget(btn)
        
        # Sıfırla Butonu
        btn_reset = QPushButton("SIFIRLA (HOME)")
        btn_reset.clicked.connect(self.reset_positions)
        layout.addWidget(btn_reset)

        self.setLayout(layout)

    def update_labels(self):
        for i, slider in enumerate(self.sliders):
            val = slider.value() / 100.0
            self.labels[i].setText(f"Joint {i+1}: {val:.2f} rad")

    def reset_positions(self):
        for slider in self.sliders:
            slider.setValue(0)
        self.send_command()

    def send_command(self):
        # Mesajı hazırla
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        # Slider değerlerini al (100'e bölerek radyana çevir)
        point.positions = [s.value() / 100.0 for s in self.sliders]
        point.time_from_start.sec = 2  # Hareket 2 saniye sürsün (Yumuşak geçiş)
        
        msg.points.append(point)
        
        # Gönder!
        self.publisher.publish(msg)
        print(f"Komut Gönderildi: {point.positions}")

    def closeEvent(self, event):
        # Kapatırken ROS'u temizle
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RobotArmUI()
    window.show()
    sys.exit(app.exec_())