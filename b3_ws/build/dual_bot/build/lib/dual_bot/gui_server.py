import sys
from PyQt5.QtGui import QPixmap, QImage
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from PyQt5.QtCore import Qt, QTimer, QTime, pyqtSignal, QObject
from PyQt5.QtWidgets import (
    QApplication, QWidget, QMainWindow, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QLineEdit, QGroupBox, QButtonGroup, QStackedWidget, QComboBox)
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
import threading
from rclpy.executors import MultiThreadedExecutor
import cv2
import numpy as np
import json
import os
from PySide2.QtCore import *
from PySide2.QtWidgets import *
from PySide2.QtGui import *
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml
import PIL.Image
import PIL.ImageDraw



class GUIserver(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # main init
        self.setWindowTitle("Cleaning Bot Duo")
        self.setGeometry(100, 100, 1600, 1000)
        self.node = Node("gui_node")
        self.node.bridge = CvBridge()
        self.trash_count = 0
        self.wall_e_status = "Idle"
        self.eve_status = "Idle"
        
        # Map related initialization
        self.map_image_path = '/home/viator/ws/walleva_ws/map/map.pgm'
        self.map_yaml_path = '/home/viator/ws/walleva_ws/map/map.yaml'
        self.load_map()
        self.amcl_pose_x = 0
        self.amcl_pose_y = 0
        self.dot_size = 2
        
        # qos
        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST
        qos.durability = DurabilityPolicy.VOLATILE
        
        # publisher
        self.pub_emergency_stop = self.node.create_publisher(
            String,
            "emergency_stop",
            10
        )
        self.pub_start_stop_toggle = self.node.create_publisher(
            String,
            "start_stop_toggle",
            10
        )
        
        # subscriber
        self.sub_wall_e_eye_image = self.node.create_subscription(
            Image,
            "/tb1/camera/image_raw",
            self.wall_e_eye_image_callback,
            qos
        )
        self.sub_global_eye_wall_e = self.node.create_subscription(
            PoseWithCovarianceStamped,
            "/tb1/amcl_pose",
            self.amcl_pose_callback_wall_e,
            10
        )
        self.sub_global_eye_eve = self.node.create_subscription(
            PoseWithCovarianceStamped,
            "/tb2/amcl_pose",
            self.amcl_pose_callback_eve,
            10
        )
        self.sub_eve_eye_image = self.node.create_subscription(
            Image,
            "/tb2/camera/image_raw",
            self.eve_eye_image_callback,
            qos
        )
        # sub_wall_e_status
        # sub_trash_count
        # sub_eve_status
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(10)
        
        # Main widget
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)

        # Top section
        eye_layout = QHBoxLayout()
        
        wall_e_eye_layout = QVBoxLayout()
        wall_e_eye_label = QLabel("Wall-E Eye")
        wall_e_eye_label.setAlignment(Qt.AlignCenter)
        wall_e_eye_label.setFixedHeight(30)
        wall_e_eye_label.setStyleSheet("font-size: 20px; font-weight: bold;")
        self.wall_e_eye_display = QLabel()
        self.wall_e_eye_display.setStyleSheet("border: 2px solid black; background-color: black;")
        self.wall_e_eye_display.setFixedSize(500, 500)
        wall_e_eye_layout.addWidget(wall_e_eye_label)
        wall_e_eye_layout.addWidget(self.wall_e_eye_display)

        global_eye_layout = QVBoxLayout()
        global_eye_label = QLabel("Minimap")
        global_eye_label.setAlignment(Qt.AlignCenter)
        global_eye_label.setFixedHeight(30)
        global_eye_label.setStyleSheet("font-size: 20px; font-weight: bold;")
        self.global_eye_display = QLabel()
        self.global_eye_display.setStyleSheet("border: 2px solid black; background-color: black;")
        self.global_eye_display.setFixedSize(500, 500)
        global_eye_layout.addWidget(global_eye_label)
        global_eye_layout.addWidget(self.global_eye_display)

        eve_eye_layout = QVBoxLayout()
        eve_eye_label = QLabel("Eve Eye")
        eve_eye_label.setAlignment(Qt.AlignCenter)
        eve_eye_label.setFixedHeight(30)
        eve_eye_label.setStyleSheet("font-size: 20px; font-weight: bold;")
        self.eve_eye_display = QLabel()
        self.eve_eye_display.setStyleSheet("border: 2px solid black; background-color: black;")
        self.eve_eye_display.setFixedSize(500, 500)
        eve_eye_layout.addWidget(eve_eye_label)
        eve_eye_layout.addWidget(self.eve_eye_display)
        
        eye_layout.addLayout(wall_e_eye_layout)
        eye_layout.addLayout(global_eye_layout)
        eye_layout.addLayout(eve_eye_layout)
        main_layout.addLayout(eye_layout)

        # Bottom section
        control_layout = QVBoxLayout()
        self.create_control_buttons(control_layout)
        main_layout.addLayout(control_layout)
        
        # sub init
        self.start_stop_toggle.setChecked(False)
        self.start_stop_toggle.setText("Start")
        
    def load_map(self):
        image = PIL.Image.open(self.map_image_path)
        self.map_width, self.map_height = image.size
        self.image_rgb = image.convert('RGB')

        with open(self.map_yaml_path, 'r') as file:
            data = yaml.safe_load(file)

        self.resolution = data['resolution']
        self.map_origin_x = -data['origin'][0]
        self.map_origin_y = data['origin'][1] + self.map_height * self.resolution

    def amcl_pose_callback_wall_e(self, msg):
        self.amcl_pose_x_wall_e = msg.pose.pose.position.x
        self.amcl_pose_y_wall_e = msg.pose.pose.position.y
        # self.node.get_logger().info(f'Received AMCL pose: x={self.amcl_pose_x}, y={self.amcl_pose_y}')
        self.update_minimap()
    
    def amcl_pose_callback_eve(self, msg):
        self.amcl_pose_x_eve = msg.pose.pose.position.x
        self.amcl_pose_y_eve = msg.pose.pose.position.y
        # self.node.get_logger().info(f'Received AMCL pose: x={self.amcl_pose_x}, y={self.amcl_pose_y}')
        self.update_minimap()
    
    def update_minimap(self):
        x_pixel_wall_e = (self.map_origin_x + self.amcl_pose_x_wall_e) / self.resolution
        y_pixel_wall_e = (self.map_origin_y - self.amcl_pose_y_wall_e) / self.resolution
        x_pixel_eve = (self.map_origin_x + self.amcl_pose_x_eve) / self.resolution
        y_pixel_eve = (self.map_origin_y - self.amcl_pose_y_eve) / self.resolution

        image_copy = self.image_rgb.copy()
        draw = PIL.ImageDraw.Draw(image_copy)
        draw.ellipse(
            (x_pixel_wall_e - self.dot_size, y_pixel_wall_e - self.dot_size, x_pixel_wall_e + self.dot_size, y_pixel_wall_e + self.dot_size),
            fill='red'
        )
        draw.ellipse(
            (x_pixel_eve - self.dot_size, y_pixel_eve - self.dot_size, x_pixel_eve + self.dot_size, y_pixel_eve + self.dot_size),
            fill='blue'
        )
        image_rotated = image_copy.rotate(90, expand=True)
        image_resized = image_rotated.resize((500, 500))
        pil_image = image_resized.convert('RGBA')
        data = pil_image.tobytes("raw", "RGBA")
        qimage = QImage(data, 500, 500, QImage.Format_RGBA8888)
        pixmap = QPixmap.fromImage(qimage)
        self.global_eye_display.setPixmap(pixmap)
        
    def create_control_buttons(self, layout):
        self.emergency_stop_button = QPushButton("Emergency Stop")
        self.emergency_stop_button.setStyleSheet("font-size: 30px; background-color: red; color: white;")
        self.emergency_stop_button.setFixedHeight(100)
        self.emergency_stop_button.clicked.connect(self.button_emergency_stop)
        layout.addWidget(self.emergency_stop_button)
        
        self.start_stop_toggle = QPushButton("Start")
        self.start_stop_toggle.setStyleSheet("font-size: 30px;")
        self.start_stop_toggle.setCheckable(True)
        self.start_stop_toggle.setFixedHeight(100)
        self.start_stop_toggle.clicked.connect(self.toggle_start_stop)
        layout.addWidget(self.start_stop_toggle)

        horizontal_layout = QHBoxLayout()

        self.wall_e_status_label = QLabel(f"Wall-E Status\n\n(  {self.wall_e_status}  )")
        self.wall_e_status_label.setStyleSheet("font-size: 30px;")
        self.wall_e_status_label.setAlignment(Qt.AlignCenter)
        self.wall_e_status_label.setFixedHeight(200)
        horizontal_layout.addWidget(self.wall_e_status_label)

        self.trash_count_label = QLabel(f"Trash Count\n\n(  {self.trash_count}  )")
        self.trash_count_label.setStyleSheet("font-size: 30px;")
        self.trash_count_label.setAlignment(Qt.AlignCenter)
        self.trash_count_label.setFixedHeight(200)
        horizontal_layout.addWidget(self.trash_count_label)

        self.eve_status_label = QLabel(f"Eve Status\n\n(  {self.eve_status}  )")
        self.eve_status_label.setStyleSheet("font-size: 30px;")
        self.eve_status_label.setAlignment(Qt.AlignCenter)
        self.eve_status_label.setFixedHeight(200)
        horizontal_layout.addWidget(self.eve_status_label)

        layout.addLayout(horizontal_layout)
    
    def button_emergency_stop(self):
        self.start_stop_toggle.setChecked(False)
        self.start_stop_toggle.setText("Start")
        msg = String()
        msg.data = "Emergency Stop"
        self.pub_emergency_stop.publish(msg)
        self.node.get_logger().warning("Emergency Stop")
    
    def toggle_start_stop(self):
        if self.start_stop_toggle.isChecked():
            self.start_stop_toggle.setText("Stop")
            msg = String()
            msg.data = "Start"
            self.pub_start_stop_toggle.publish(msg)
            self.node.get_logger().warning("Start")
        else:
            self.start_stop_toggle.setText("Start")
            msg = String()
            msg.data = "Stop"
            self.pub_start_stop_toggle.publish(msg)
            self.node.get_logger().warning("Stop")
    
    def wall_e_eye_image_callback(self, msg):
        try:
            cv_image = self.node.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            qt_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            self.wall_e_eye_display.setPixmap(QPixmap.fromImage(qt_image))
        except Exception as e:
            self.node.get_logger().error(f"Failed to process image: {e}")
    
    def eve_eye_image_callback(self, msg):
        try:
            cv_image = self.node.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            qt_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            self.eve_eye_display.setPixmap(QPixmap.fromImage(qt_image))
        except Exception as e:
            self.node.get_logger().error(f"Failed to process image: {e}")
    
    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0.01)

    def closeEvent(self, event):
        self.node.get_logger().info("Shutting down GUI node...")
        self.timer.stop()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        super().closeEvent(event)



def main():
    rclpy.init()
    app = QApplication(sys.argv)
    executor = MultiThreadedExecutor()
    gui_node = GUIserver()
    
    def ros_spin():
        try:
            executor.spin()
        except Exception as e:
            print(f"ROS Executor error: {e}")
        finally:
            executor.shutdown()
            if rclpy.ok():
                rclpy.shutdown()
            
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    gui_node.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == "__main__":
    main()
