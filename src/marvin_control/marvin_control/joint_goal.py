import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointGoalPublisher(Node):
    def __init__(self):
        super().__init__('joint_goal_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_goals', 10)
        self.get_logger().info('JointGoalPublisher is running...')
        self.timer = self.create_timer(1.0, self.timer_callback)  # Trigger every 5 seconds
        self.goal_index = 0
        self.joint_goals = [
            [0.0, 0.0, 0.0, 0.0, 0.0],  # goal_joint_angle_01
            [0.5, 0.0, 0.0, 0.0, 0.0]   # goal_joint_angle_02
        ]

    def timer_callback(self):
        current_goal = self.joint_goals[self.goal_index]
        self.publish_joint_goals(current_goal)
        self.goal_index = (self.goal_index + 1) % len(self.joint_goals)  # Alternate goals

    def publish_joint_goals(self, goal_joint_angle):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        msg.position = goal_joint_angle
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published Joint Goals: {msg.position}')


def main(args=None):
    rclpy.init(args=args)
    joint_goal_publisher = JointGoalPublisher()

    try:
        rclpy.spin(joint_goal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        joint_goal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
