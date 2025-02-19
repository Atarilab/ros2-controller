import rclpy
import numpy as np
from rclpy.node import Node
from robot_controller_interface.msg import RobotState, RobotActions, RobotControl
from mj_pin.abstract import Controller  # type: ignore
from mj_pin.utils import get_robot_description  # type: ignore

class ControllerNode(Node):
    def __init__(self, controller : Controller, control_freq : float = 1000):
        super().__init__('controller_node')
        self.controller = controller
        
        self.state_subscription = self.create_subscription(
            RobotState,
            '/robot_state',
            self.state_callback,
            10)
        # self.action_subscription = self.create_subscription(
        #     RobotActions, '/robot_actions', self.action_callback, 1)
        self.control_publisher = self.create_publisher(
            RobotControl, '/robot_control', 10
        )
        self._q = None
        self._v = None
        self.t = 0.
        self.control_dt = 1. / control_freq
        self.timer = self.create_timer(self.control_dt, self.send_control)
        self.get_logger().info('Controller node initialized')
        # No action server setup so far
        # self.current_actions = RobotActions()
        # self.current_actions.target_velocity = [0.3, 0., 0.]
        # self.current_actions.yaw_rate = 0.
        # self.action_callback(self.current_actions)
        
    def state_callback(self, msg):
        # self.get_logger().info('Received robot state')
        self.update_state_qv(msg)
        # self.send_control()

    # def action_callback(self, msg):
    #     self.current_actions = msg
    #     self.controller.set_command(
    #         self.current_actions.target_velocity,
    #         self.current_actions.yaw_rate
    #         )
    
    def update_state_qv(self, msg : RobotState):
        if self._q is None or self._v is None:
            njnt = len(msg.joint_state.position)
            self._q = np.zeros(7 + njnt)
            self._v = np.zeros(6 + njnt)

        self._q[0] = msg.base_state.pose.pose.position.x
        self._q[1] = msg.base_state.pose.pose.position.y
        self._q[2] = msg.base_state.pose.pose.position.z
        self._q[4] = msg.base_state.pose.pose.orientation.x
        self._q[5] = msg.base_state.pose.pose.orientation.y
        self._q[6] = msg.base_state.pose.pose.orientation.z
        self._q[3] = msg.base_state.pose.pose.orientation.w
        self._q[7:] = msg.joint_state.position
        
        self._v[0] = msg.base_state.twist.twist.linear.x
        self._v[1] = msg.base_state.twist.twist.linear.y
        self._v[2] = msg.base_state.twist.twist.linear.z
        self._v[3] = msg.base_state.twist.twist.angular.x
        self._v[4] = msg.base_state.twist.twist.angular.y
        self._v[5] = msg.base_state.twist.twist.angular.z
        self._v[6:] = msg.joint_state.velocity
        
        t_ns = self.get_clock().now().seconds_nanoseconds()
        self.t = t_ns[0] + t_ns[1] * 1.e-9
        
    def send_control(self):
        if not(self._q is None or
               self._v is None
            #    self.current_actions
               ):
            

            # self.get_logger().info(f'time since start: {t:.6f} s')
            torque_map = self.controller._get_torques(self.t, self._q, self._v)
            
            msg = RobotControl()
            for name, torque in torque_map.items():
                msg.joint_ctrl.name.append(name)
                msg.joint_ctrl.effort.append(torque)
            
            self.control_publisher.publish(msg)

def main(args=None):
    import sys
    sys.path.insert(0, '/home/atari/workspace')
    from mpc_controller.mpc import LocomotionMPC
    
    CONTROL_FREQ = 200
    ROBOT_NAME = "go2"
    robot_desc = get_robot_description(ROBOT_NAME)
    feet_frame_names = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]
    print(feet_frame_names)

    mpc_ = LocomotionMPC(
        path_urdf=robot_desc.urdf_path,
        feet_frame_names = feet_frame_names,
        robot_name=ROBOT_NAME,
        joint_ref = robot_desc.q0,
        sim_dt=1/CONTROL_FREQ,
        print_info=False,
        )
    mpc_.set_command([0.3, 0., 0.], 0.)
    print(mpc_)
    
    rclpy.init(args=args)
    node = ControllerNode(mpc_, CONTROL_FREQ)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()