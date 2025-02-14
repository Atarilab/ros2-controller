import rclpy
from rclpy.node import Node
from src.msg import RobotState
from geometry_msgs.msg import Pose
import numpy as np

class StatePublisherNode(Node):
    def __init__(self):
        super().__init__('state_publisher_node')
        self.publisher = self.create_publisher(RobotState, '/robot_state', 10)
        self.timer = self.create_timer(0.1, self.publish_state)  # 10 Hz

    def publish_state(self):
        msg = RobotState()
        msg.joint_positions = [0.0, 0.0, 0.0]  # Example data
        msg.joint_velocities = [0.0, 0.0, 0.0]  # Example data
        msg.joint_torques = [0.0, 0.0, 0.0]  # Example data
        msg.end_effector_pose = Pose()  # Example data
        self.publisher.publish(msg)
        self.get_logger().info('Publishing robot state')

def main(args=None):
    rclpy.init(args=args)
    node = StatePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()