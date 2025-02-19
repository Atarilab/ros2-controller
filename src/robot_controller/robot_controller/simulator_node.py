import rclpy
from rclpy.node import Node
from robot_controller_interface.msg import RobotControl, RobotState
from geometry_msgs.msg import Point, Quaternion
import mujoco
import mujoco.viewer
import numpy as np
import threading
import time
from mj_pin.utils import get_robot_description, mj_joint_name2act_id, mj_joint_name2dof

class SimulatorNode(Node):
    def __init__(self, xml_path : str, sim_freq : int = 1000):
        super().__init__('simulator_node')
        self.subscription = self.create_subscription(
            RobotControl,
            '/robot_control',
            self.control_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(RobotState, '/robot_state', 10)
        
        self.sim_dt = 1. / sim_freq
        self.timer = self.create_timer(self.sim_dt, self.publish_state)  # 10 Hz
        
        self.mj_model = mujoco.MjModel.from_xml_path(xml_path)
        self.mj_model.opt.timestep = self.sim_dt
        self.nu = self.mj_model.nu
        
        self.mj_data = mujoco.MjData(self.mj_model)
        mujoco.mj_resetDataKeyframe(self.mj_model, self.mj_data, 0)
        
        self.joint_name2act_id = mj_joint_name2act_id(self.mj_model)
        self.joint_name2dof = mj_joint_name2dof(self.mj_model)
        self.last_ctrl = RobotControl()
        
        self.stop = False
        try:
            physics_thread = threading.Thread(target=self._run_physics)
            physics_thread.start()

        except KeyboardInterrupt:
            self.stop = True
            if physics_thread and physics_thread.is_alive():
                physics_thread.join()
                
    def set_ctrl(self, msg : RobotControl):
        for name, torque in zip(msg.joint_ctrl.name, msg.joint_ctrl.effort):
            if name in self.joint_name2act_id:
                self.mj_data.ctrl[self.joint_name2act_id[name]] = torque

    def _run_physics(self):
        while True:
            if self.stop:
                break
            
            t = time.time()
            mujoco.mj_step1(self.mj_model, self.mj_data)
            self.set_ctrl(self.last_ctrl)
            mujoco.mj_step2(self.mj_model, self.mj_data)
            physics_time = time.time() - t
            
            time.sleep(max(0, self.sim_dt - physics_time))
            
        self.stop = True
                
    def control_callback(self, msg : RobotControl):
        # self.get_logger().info(f'Received robot state: {msg}')
        self.last_ctrl = msg
        
    def publish_state(self):
        
        msg = RobotState()
        msg.base_state.pose.pose.position = Point(
            x=self.mj_data.qpos[0],
            y=self.mj_data.qpos[1],
            z=self.mj_data.qpos[2],
        )
        msg.base_state.pose.pose.orientation = Quaternion(
            x=self.mj_data.qpos[4],
            y=self.mj_data.qpos[5],
            z=self.mj_data.qpos[6],
            w=self.mj_data.qpos[3],
        )
        
        diff = len(self.mj_data.qpos) - len(self.mj_data.qvel)
        for name, dof in self.joint_name2dof.items():
            if name in self.joint_name2act_id:
                msg.joint_state.name.append(name)
                msg.joint_state.position.append(self.mj_data.qpos[dof+diff])
                msg.joint_state.velocity.append(self.mj_data.qpos[dof+diff])

        msg.base_state.twist.twist.linear.x = self.mj_data.qvel[0]
        msg.base_state.twist.twist.linear.y = self.mj_data.qvel[1]
        msg.base_state.twist.twist.linear.z = self.mj_data.qvel[2]
        msg.base_state.twist.twist.angular.x = self.mj_data.qvel[3]
        msg.base_state.twist.twist.angular.y = self.mj_data.qvel[4]
        msg.base_state.twist.twist.angular.z = self.mj_data.qvel[5]
        msg.joint_state.velocity = self.mj_data.qvel[-self.nu:].tolist()
        
        self.publisher.publish(msg)

        
def main(args=None):
    desc = get_robot_description("go2")
    SIM_FREQ = 200
    time.sleep(2.5)
    rclpy.init(args=args)
    node = SimulatorNode(desc.xml_scene_path, sim_freq=SIM_FREQ)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()