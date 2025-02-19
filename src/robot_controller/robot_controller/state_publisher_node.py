import rclpy
from rclpy.node import Node
from robot_controller_interface.msg import RobotState
from geometry_msgs.msg import Point, Quaternion
import numpy as np
import time

class StatePublisherNode(Node):
    def __init__(self, freq=30):
        super().__init__('state_publisher_node')
        self.publisher = self.create_publisher(RobotState, '/robot_state', 10)
        self.freq = freq
        self.timer = self.create_timer(1 / self.freq, self.publish_state)  # 10 Hz

    def publish_state(self):
        current_time = time.time()
        sine_value = 0.5 * np.sin(2 * np.pi * 0.5 * current_time)
        
        msg = RobotState()
        msg.base_state.pose.pose.position = Point(
            x=sine_value * 0.5,
            y=sine_value * 0.5,
            z=sine_value * 0.1 + 0.3
        )
        msg.base_state.pose.pose.orientation = Quaternion(
            x=0.,
            y=0.,
            z=0.,
            w=1.
        )
        msg.joint_state.position = [sine_value] * 12
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StatePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()