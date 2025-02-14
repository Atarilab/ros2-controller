import rclpy
from rclpy.node import Node
from src.msg import RobotActions

class ActionPublisherNode(Node):
    def __init__(self):
        super().__init__('action_publisher_node')
        self.publisher = self.create_publisher(RobotActions, '/robot_actions', 10)
        self.timer = self.create_timer(0.1, self.publish_actions)  # 10 Hz

    def publish_actions(self):
        msg = RobotActions()
        msg.target_joint_positions = [1.0, 1.0, 1.0]  # Example data
        msg.target_joint_velocities = [0.0, 0.0, 0.0]  # Example data
        msg.target_joint_torques = [0.0, 0.0, 0.0]  # Example data
        self.publisher.publish(msg)
        self.get_logger().info('Publishing robot actions')

def main(args=None):
    rclpy.init(args=args)
    node = ActionPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()