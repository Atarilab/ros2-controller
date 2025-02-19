import rclpy
from rclpy.node import Node
from robot_controller_interface.msg import RobotState
import mujoco
import mujoco.viewer
import numpy as np
import threading
import time
from mj_pin.utils import get_robot_description

class StateSubscriberNode(Node):
    def __init__(self):
        super().__init__('state_subscriber_node')
        self.subscription = self.create_subscription(
            RobotState,
            '/robot_state',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
                
    def listener_callback(self, msg):
        # self.get_logger().info(f'Received robot state: {msg}')
        pass

class ViewerNode(StateSubscriberNode):
    def __init__(self, xml_path : str, viewer_freq : int = 30):
        super().__init__()
        
        self.mj_model = mujoco.MjModel.from_xml_path(xml_path)
        self.mj_data = mujoco.MjData(self.mj_model)
        self.nu = self.mj_model.nu
        self.stop_viewer = False
        self.viewer_freq = viewer_freq

        try:
            viewer_thread = threading.Thread(target=self._run_viewer)
            viewer_thread.start()

        except KeyboardInterrupt:
            self.stop_viewer = True
            if viewer_thread and viewer_thread.is_alive():
                viewer_thread.join()

    def _run_viewer(self):
        viewer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data, show_left_ui=False, show_right_ui=False)
        
        while viewer.is_running():
            if self.stop_viewer:
                break
            t = time.time()
            viewer.sync()
            render_time = time.time() - t
            time.sleep(max(0, 1 / self.viewer_freq - render_time))
            
        self.stop_viewer = True
        viewer.close()
                
    def listener_callback(self, msg):
        super().listener_callback(msg)
        # self.get_logger().info(f'Received robot state: {msg}')
        self.mj_data.qpos[-self.nu:] = np.array(msg.joint_state.position)
        self.mj_data.qpos[0] = msg.base_state.pose.pose.position.x
        self.mj_data.qpos[1] = msg.base_state.pose.pose.position.y
        self.mj_data.qpos[2] = msg.base_state.pose.pose.position.z
        self.mj_data.qpos[3] = msg.base_state.pose.pose.orientation.x
        self.mj_data.qpos[4] = msg.base_state.pose.pose.orientation.y
        self.mj_data.qpos[5] = msg.base_state.pose.pose.orientation.z
        self.mj_data.qpos[6] = msg.base_state.pose.pose.orientation.w
        
        mujoco.mj_forward(self.mj_model, self.mj_data)
        
def main(args=None):
    desc = get_robot_description("go2")
    rclpy.init(args=args)
    node = ViewerNode(desc.xml_scene_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()