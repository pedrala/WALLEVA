import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
import math
import time

class OpenManipulatorWithGripper(Node):
    def __init__(self):
        super().__init__('open_manipulator_with_gripper')

        # Initialize publishers and action clients
        self.joint_publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_action_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')

        # Constants for manipulator arm
        self.r1 = 130
        self.r2 = 124
        self.r3 = 126
        self.th1_offset = -math.atan2(0.024, 0.128)
        self.th2_offset = -0.5 * math.pi - self.th1_offset

        # Initialize trajectory message
        self.trajectory_msg = JointTrajectory()
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

    def update_position(self, x, y, z):
        """Calculate joint angles and update trajectory message."""
        try:
            sr1, sr2, sr3, Sxy = self.solv_robot_arm2(x, y, z, self.r1, self.r2, self.r3)
            joint_angles = [Sxy, sr1 + self.th1_offset, sr2 + self.th2_offset, sr3]

            # Update trajectory point
            point = JointTrajectoryPoint()
            point.positions = joint_angles
            point.velocities = [0.0] * 4
            point.time_from_start.sec = 3
            self.trajectory_msg.points = [point]

            # Publish trajectory
            self.joint_publisher.publish(self.trajectory_msg)
            self.get_logger().info(f"Published trajectory with joint angles: {joint_angles}")

        except ValueError as e:
            self.get_logger().error(f"Error calculating joint angles: {e}")

    def control_gripper(self, position):
        """Send a command to control the gripper asynchronously."""
        goal = GripperCommand.Goal()
        goal.command.position = position

        # Log and send the goal asynchronously
        self.get_logger().info(f"Sending gripper command to position: {position}")
        future = self.gripper_action_client.send_goal_async(goal)
        future.add_done_callback(self._handle_gripper_goal_response)

    def _handle_gripper_goal_response(self, future):
        """Handle the response for the gripper goal."""
        goal_handle = future.result()

        if not goal_handle:
            self.get_logger().error("Failed to send gripper command.")
            return

        if not goal_handle.accepted:
            self.get_logger().error("Gripper command was not accepted.")
            return

        self.get_logger().info("Gripper command accepted, waiting for result.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._handle_gripper_result)

    def _handle_gripper_result(self, future):
        """Handle the result of the gripper action."""
        result = future.result()

        if result.result:
            self.get_logger().info("Gripper successfully moved to the desired position.")
        else:
            self.get_logger().error("Failed to execute gripper command.")

    def solv_robot_arm2(self, x, y, z, r1, r2, r3):
        """Calculate joint angles for the robot arm to reach the given (x, y, z)."""
        Rt = math.sqrt(x**2 + y**2 + z**2)
        if Rt > (r1 + r2 + r3):
            raise ValueError("Target position is out of the manipulator's reach.")

        St = math.asin(z / Rt)
        Sxy = math.atan2(y, x)
        s1, s2 = self.solv2(r1, r2, Rt)
        sr1 = math.pi / 2 - (s1 + St)
        sr2 = s1 + s2
        sr3 = math.pi - (sr1 + sr2)
        return sr1, sr2, sr3, Sxy

    def solv2(self, r1, r2, r3):
        """Calculate angles between links."""
        d1 = (r3**2 - r2**2 + r1**2) / (2 * r3)
        d2 = (r3**2 + r2**2 - r1**2) / (2 * r3)
        s1 = math.acos(d1 / r1)
        s2 = math.acos(d2 / r2)
        return s1, s2

def main():
    rclpy.init()
    node = OpenManipulatorWithGripper()

    try:
        # Move the manipulator to a target position
        target_position = [200, 120, 50]
        time.sleep(3)

        node.control_gripper(0.025)
        time.sleep(3)
        node.update_position(target_position[0], target_position[1], target_position[2])
        time.sleep(3)
        node.control_gripper(-0.015)
        time.sleep(3)
        target_position = [150, -120, 70]
        node.update_position(target_position[0], target_position[1], target_position[2])
        # Keep spinning to ensure callbacks are processed
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
