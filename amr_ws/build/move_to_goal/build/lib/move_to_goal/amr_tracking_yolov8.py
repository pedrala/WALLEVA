import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
import math
import time

class AMRTrackingYOLOv8(Node):
    def __init__(self):
        super().__init__('amr_tracking_yolov8')
        self.bridge = CvBridge()
        self.model = YOLO('best.pt')  # YOLOv8 모델 로드
        self.first_object_id = None  # 첫 번째 객체 ID 저장
        self.first_object_box = None  # 첫 번째 객체 바운딩 박스 저장
        
        # 콜백 실행 제한 설정
        self.last_callback_time = 0.0
        self.callback_interval = 0.5  # 최소 실행 간격 (초)
        self.get_logger().info("AMR Tracking YOLOv8 Node Initialized.")  # 초기화 로그
        
        self.Kp = 0.001
        self.Ki = 0.0001
        self.Kd = 0.01

        
        # Waypoints 정의
        self.way_point_coordinates = [
            {"way_point_number": 1, "x": -0.0266, "y": -0.0388, "yaw": -0.785},
            {"way_point_number": 2, "x": 2.6459, "y": -2.4964, "yaw": -0.0},
            {"way_point_number": 3, "x": 7.6131, "y": -2.4085, "yaw": 0.785},
            {"way_point_number": 4, "x": 9.7074, "y": -0.0979, "yaw": 0.0},
            {"way_point_number": 5, "x": 9.6343, "y": 4.0883, "yaw": 2.355},
            {"way_point_number": 6, "x": 7.831, "y": 6.3066, "yaw": 3.14},
            {"way_point_number": 7, "x": 2.3753, "y": 6.4383, "yaw": -2.355},
            {"way_point_number": 8, "x": -0.0653, "y": 3.6761, "yaw": -1.57}
        ]
        self.current_waypoint_index = 0  # 초기 위치는 number 1
        self.target_distance = 0.2  # 라이다 목표 거리 (20cm)

        # 상태 플래그
        self.is_moving_to_waypoint = True
        self.is_tracking = False
        self.is_adjusting_distance = False

        # ActionClient 초기화
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.goal_handle = None  # 현재 목표를 관리할 핸들

        # 구독 및 퍼블리셔 설정
        self.create_subscription(Image, '/pi_camera/image_raw', self.camera_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("AMR Tracking YOLOv8 Node Started.")
        self.move_to_next_waypoint()

    def move_to_next_waypoint(self):
        if self.current_waypoint_index >= len(self.way_point_coordinates):
            self.get_logger().info("All waypoints reached. Stopping.")
            return

        waypoint = self.way_point_coordinates[self.current_waypoint_index]
        self.get_logger().info(f"Navigating to waypoint {self.current_waypoint_index + 1}: {waypoint}")

        # NavigateToPose 메시지 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint["x"]
        goal_msg.pose.pose.position.y = waypoint["y"]
        goal_msg.pose.pose.orientation.z = math.sin(waypoint["yaw"] / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(waypoint["yaw"] / 2.0)

        self.action_client.wait_for_server()
        self.action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected by server.")
            return

        self.get_logger().info("Goal accepted by server.")
        self.goal_handle = goal_handle  # 현재 목표 핸들 저장
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def cancel_current_goal(self):
        if self.goal_handle:
            self.get_logger().info("Cancelling current goal...")
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.goal_cancelled_callback)

    def goal_cancelled_callback(self, future):
        cancel_response = future.result()
        if cancel_response.return_code == 0:  # CANCEL_GOAL_SUCCESS
            self.get_logger().info("Goal successfully cancelled.")
        else:
            self.get_logger().info(f"Failed to cancel goal. Return code: {cancel_response.return_code}")
        self.goal_handle = None  # 목표 핸들 초기화


    def result_callback(self, future):
        result = future.result()
        if result.status == 3:  # STATUS_SUCCEEDED
            self.get_logger().info(f"Waypoint {self.current_waypoint_index + 1} reached.")
            self.current_waypoint_index += 1
            self.move_to_next_waypoint()
        else:
            self.get_logger().info("Goal failed or canceled.")
        self.goal_handle = None  # 목표 핸들 초기화

    def camera_callback(self, image_msg):
        """카메라 데이터를 사용하여 YOLO로 객체 감지 및 제어."""
        # 현재 시간과 마지막 실행 시간을 비교하여 제한
        current_time = time.time()
        if current_time - self.last_callback_time < self.callback_interval:
            return  # 제한 시간 미만이면 실행하지 않음
        self.last_callback_time = current_time

        # 상태 확인
        if not self.is_moving_to_waypoint and not self.is_tracking and not self.is_adjusting_distance:
            return

        self.get_logger().info(f"is_moving_to_waypoint: {self.is_moving_to_waypoint}")
        self.get_logger().info(f"is_tracking: {self.is_tracking}")
        self.get_logger().info(f"is_adjusting_distance: {self.is_adjusting_distance}")

        try:
            # 이미지 처리 및 YOLO 추론
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            results = self.model(cv_image, verbose=False)
            boxes = results[0].boxes.data.cpu().numpy()

            if len(boxes) == 0:
                self.get_logger().info("No objects detected.")
                self.first_object_box = None  # 객체가 없으면 초기화
                return

            # 첫 번째 객체 설정
            if self.first_object_box is None:
                self.first_object_box = boxes[0]
                self.is_tracking = True  # 객체 감지 시 트래킹 시작
                self.is_moving_to_waypoint = False  # 웨이포인트 이동 중단
                self.get_logger().info("First object detected and locked. Switching to tracking mode.")
            else:
                x1, y1, x2, y2, _, _ = self.first_object_box
                # 객체가 계속 프레임 안에 있는지 확인
                if not any((x1 <= box[0] <= x2 and y1 <= box[1] <= y2) for box in boxes):
                    self.get_logger().info("First object lost. Resetting lock.")
                    self.first_object_box = None
                    self.is_tracking = False
                    return

            # 첫 번째 객체 중심 계산
            x1, y1, x2, y2, _, _ = self.first_object_box
            cx = (x1 + x2) // 2
            image_center = cv_image.shape[1] // 2
            error = image_center - cx

            # PID 제어 계산
            self.integral_error += error
            self.integral_error = max(min(self.integral_error, 1000), -1000)  # Integral 제한
            derivative_error = error - self.prev_error
            angular_error = (self.Kp * error) + (self.Ki * self.integral_error) + (self.Kd * derivative_error)
            self.prev_error = error

            # 최소 회전 속도 설정
            if abs(angular_error) < 0.1:
                angular_error = 0.1 if angular_error > 0 else -0.1

            # Twist 메시지 생성 (선형 이동 없이 좌우 회전만 수행)
            twist = Twist()
            twist.angular.z = angular_error
            twist.linear.x = 0.0  # 항상 0으로 설정하여 전진하지 않음
            self.cmd_vel_pub.publish(twist)

            self.get_logger().info(f"Angular Error: {angular_error}, Linear Velocity: {twist.linear.x}")

            # 객체 중심에 위치
            if abs(cx - image_center) < 50:  # 임계값 완화
                self.get_logger().info("Object centered. Switching to distance adjustment.")
                self.is_tracking = False
                self.is_adjusting_distance = True
                self.first_object_box = None  # 첫 번째 객체 초기화
                self.current_twist = Twist()
                self.cmd_vel_pub.publish(self.current_twist)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")





    def lidar_callback(self, scan_msg):
        if not self.is_adjusting_distance:
            return  # 거리 조정 중이 아니면 무시

        min_distance = min(scan_msg.ranges)  # LiDAR 데이터에서 최소 거리 추출
        self.get_logger().info(f"Minimum distance to object: {min_distance} m")

        # 거리 조정 로직
        twist = Twist()
        if min_distance > self.target_distance + 0.02:
            self.get_logger().info("Too far. Moving closer.")
            twist.linear.x = 0.1  # 천천히 전진
        elif min_distance < self.target_distance - 0.02:
            self.get_logger().info("Too close. Moving backward.")
            twist.linear.x = -0.1  # 천천히 후진
        else:
            self.get_logger().info("Target distance achieved. Resuming waypoint navigation.")
            self.is_adjusting_distance = False
            self.is_moving_to_waypoint = True  # 웨이포인트 이동 재개

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AMRTrackingYOLOv8()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly.")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
