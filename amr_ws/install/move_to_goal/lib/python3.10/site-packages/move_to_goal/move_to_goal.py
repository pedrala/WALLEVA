import os
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from time import sleep
from scipy.spatial.transform import Rotation as R
from math import cos, sin
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped



way_point_coordinates = [
    {"way_point_number": 1, "x": -0.0266, "y": -0.0388, "yaw": -0.785},
    {"way_point_number": 2, "x": 2.6459, "y": -2.4964, "yaw": -0.0},
    {"way_point_number": 3, "x": 7.6131, "y": -2.4085, "yaw": 0.785},
    {"way_point_number": 4, "x": 9.7074, "y": -0.0979, "yaw": 0.0},
    {"way_point_number": 5, "x": 9.6343, "y": 4.0883, "yaw": 2.355},
    {"way_point_number": 6, "x": 7.831, "y": 6.3066, "yaw": 3.14},
    {"way_point_number": 7, "x": 2.3753, "y": 6.4383, "yaw": -2.355},
    {"way_point_number": 8, "x": -0.0653, "y": 3.6761, "yaw": -1.57}
]



class MoveToGoal(Node):
    def __init__(self, namespace=''):
        super().__init__('move_to_goal_node')

        # 네임스페이스를 추가하여 NavigateToPose 액션 서버에 연결
        self._client = ActionClient(
            self,
            NavigateToPose,
            f'{namespace}/navigate_to_pose'  # 네임스페이스를 동적으로 할당
        )
        self.namespace = namespace
        self.current_goal_complete = False
        self.start_mission = False
        self.stop_requested = False

        # 첫 번째 로봇 위치 추적 (tb1/amcl_pose)
        if namespace == 'tb2':
            self.leader_pose_x = 0.0
            self.leader_pose_y = 0.0
            self.leader_yaw = 0.0
            self.create_subscription(
                PoseWithCovarianceStamped,
                '/tb1/amcl_pose',
                self.leader_pose_callback,
                10
            )

        # Start/Stop Toggle 구독
        self.create_subscription(
            String,
            'start_stop_toggle',
            self.start_stop_callback,
            10
        )

    def leader_pose_callback(self, msg):
        self.leader_pose_x = msg.pose.pose.position.x
        self.leader_pose_y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        self.leader_yaw = R.from_quat([
            quaternion.x, quaternion.y, quaternion.z, quaternion.w
        ]).as_euler('xyz')[2]

    def start_stop_callback(self, msg):
        if msg.data == "Start":
            self.get_logger().info("Start signal received. Beginning mission.")
            self.start_mission = True
            self.stop_requested = False
        elif msg.data == "Stop":
            self.get_logger().info("Stop signal received. Halting mission after current loop.")
            self.stop_requested = True

    def send_goal(self, way_point_number=None, follow_tb1=False):
        # 목표 위치 설정
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Waypoint 좌표 가져오기
        if not follow_tb1:
            coordinates = next(
                (coord for coord in way_point_coordinates if coord["way_point_number"] == way_point_number), None
            )
        else:  # tb2 follows tb1
            coordinates = {
                "x": self.leader_pose_x - 0.75 * cos(self.leader_yaw),
                "y": self.leader_pose_y - 0.75 * sin(self.leader_yaw),
                "yaw": self.leader_yaw
            }

        if coordinates is None:
            self.get_logger().error(f"웨이포인트 {way_point_number}를 찾을 수 없습니다.")
            return

        # 좌표 설정
        goal_msg.pose.pose.position.x = coordinates["x"]
        goal_msg.pose.pose.position.y = coordinates["y"]

        # Yaw를 기반으로 쿼터니언 계산
        yaw = coordinates["yaw"]
        quaternion = R.from_euler('z', yaw).as_quat()
        goal_msg.pose.pose.orientation.x = quaternion[0]
        goal_msg.pose.pose.orientation.y = quaternion[1]
        goal_msg.pose.pose.orientation.z = quaternion[2]
        goal_msg.pose.pose.orientation.w = quaternion[3]

        # 액션 서버 준비 확인 후 목표 전송
        self._client.wait_for_server()
        self.current_goal_complete = False
        future = self._client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.goal_response_callback(f, way_point_number))

        self.get_logger().info(
            f"Goal sent to {self.namespace}: way_point_number={way_point_number}, x={coordinates['x']}, y={coordinates['y']}, yaw={yaw}.")

    def goal_response_callback(self, future, way_point_number):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f"Goal for Way Point {way_point_number} was rejected.")
            return

        self.get_logger().info(f"Goal for Way Point {way_point_number} was accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.goal_result_callback(f, way_point_number))

    def goal_result_callback(self, future, way_point_number):
        result = future.result().result
        if result:
            self.get_logger().info(f"Successfully arrived at Way Point {way_point_number}.")
        else:
            self.get_logger().info(f"Failed to reach Way Point {way_point_number}.")
        self.current_goal_complete = True

    def wait_for_goal_completion(self):
        while not self.current_goal_complete:
            rclpy.spin_once(self, timeout_sec=0.1)



def main(args=None):
    rclpy.init(args=args)

    move_to_goal_tb1 = MoveToGoal(namespace='tb1')
    move_to_goal_tb2 = MoveToGoal(namespace='tb2')

    try:
        # Start 신호를 받을 때까지 대기
        move_to_goal_tb1.get_logger().info("Waiting for Start signal...")
        while not move_to_goal_tb1.start_mission:
            rclpy.spin_once(move_to_goal_tb1, timeout_sec=0.1)

        # 현재 tb1의 목표를 저장하여 중복 목표 전송 방지
        current_goal_tb1 = None

        # Waypoints 1번부터 8번까지 무한으로 반복
        while move_to_goal_tb1.start_mission:
            for way_point_number in [1, 2, 3, 4, 5, 6, 7, 8]:
                # tb1 목표 전송 (중복 전송 방지)
                if current_goal_tb1 != way_point_number:
                    move_to_goal_tb1.send_goal(way_point_number)
                    current_goal_tb1 = way_point_number

                # tb1이 목표로 이동하는 동안 tb2가 실시간으로 따라감
                while not move_to_goal_tb1.current_goal_complete:
                    # tb2의 목표를 갱신하지 않고 tb1의 뒤를 따라가도록 설정
                    move_to_goal_tb2.send_goal(follow_tb1=True)
                    rclpy.spin_once(move_to_goal_tb1, timeout_sec=0.05)
                    rclpy.spin_once(move_to_goal_tb2, timeout_sec=0.05)

                # Stop 신호를 받으면 남은 경로를 돌고 루프 종료
                if move_to_goal_tb1.stop_requested:
                    break

            if move_to_goal_tb1.stop_requested:
                break

        # Stop 신호를 받으면 남은 경로를 수행 후 1번 Waypoint로 복귀
        current_index = [1, 2, 3, 4, 5, 6, 7, 8].index(current_goal_tb1)
        remaining_waypoints = [1, 2, 3, 4, 5, 6, 7, 8][current_index + 1:]

        for way_point_number in remaining_waypoints + [1]:
            move_to_goal_tb1.send_goal(way_point_number)  # tb1의 목표 전송

            # tb1이 목표로 이동하는 동안 tb2가 실시간으로 따라감
            while not move_to_goal_tb1.current_goal_complete:
                move_to_goal_tb2.send_goal(follow_tb1=True)  # tb1의 뒤를 따라 이동
                rclpy.spin_once(move_to_goal_tb1, timeout_sec=0.05)
                rclpy.spin_once(move_to_goal_tb2, timeout_sec=0.05)

        # Stop 신호를 받은 이후 루프 종료를 보장
        move_to_goal_tb1.start_mission = False

    except KeyboardInterrupt:
        move_to_goal_tb1.get_logger().info("작업 중단.")
    finally:
        move_to_goal_tb1.destroy_node()
        move_to_goal_tb2.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
