import rclpy
from rclpy.node import Node
from robot_controller_interface.msg import RobotState
import numpy as np
import time

class StatePublisherNode(Node):
    def __init__(self, freq=30):
        super().__init__('state_publisher_node')
        self.publisher = self.create_publisher(RobotState, '/robot_state', 10)
        self.freq = freq
        self.timer = self.create_timer(1 / self.freq, self.publish_state)  # 10 Hz

    def publish_state(self):
        msg = RobotState()
        current_time = time.time()
        sine_value = 0.5 * np.sin(2 * np.pi * 0.5 * current_time)
        msg.joint_positions = [sine_value] * 12
        # msg.joint_velocities = [0.0, 0.0, 0.0]  # Example data
        # msg.joint_torques = [0.0, 0.0, 0.0]  # Example data
        self.publisher.publish(msg)
        # self.get_logger().info('Publishing robot state')

def main(args=None):
    rclpy.init(args=args)
    node = StatePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()