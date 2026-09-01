#!/usr/bin/env python3
import numpy
import rclpy
import tf_quaternion.transformations as transf

from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import geometry_msgs.msg as geometry_msgs
from nav_msgs.msg import Odometry

from plankton_utils.time import time_in_float_sec_from_msg
from bluerov2_heavy_PID.PIDRegulator import PIDRegulator
from plankton_utils.time import is_sim_time


class SingleVelocityControllerNode(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        self.configs = {}

        self.v_linear_des = numpy.zeros(3)
        self.v_angular_des = numpy.zeros(3)

        #Initialize pids with default parameters
        self.pid_vel_x = PIDRegulator(1, 0, 0, 0, 1)
        self.pid_vel_y = PIDRegulator(1, 0, 0, 0, 1)
        self.pid_vel_z = PIDRegulator(1, 0, 0, 0, 1)
        self.pid_vel_yaw = PIDRegulator(1, 0, 0, 0, 1)

        self._declare_and_fill_map("vel_x_p", 196.6673, "Kp gain for vel_x",self.configs)
        self._declare_and_fill_map("vel_x_i", 546.1016, "Ki gain for vel_x",self.configs)
        self._declare_and_fill_map("vel_x_d", -0.1657, "Kd gain for vel_x",self.configs)
        self._declare_and_fill_map("vel_x_sat", 60, "Saturation for vel_x",self.configs)
        self._declare_and_fill_map("vel_x_n", 170.8190, "Filter_coefficient",self.configs)
        self._declare_and_fill_map("vel_y_p", 264.0557, "Kp gain for vel_y",self.configs)
        self._declare_and_fill_map("vel_y_i", 653.2182, "Ki gain for vel_y",self.configs)
        self._declare_and_fill_map("vel_y_d", -3.6220, "Kd gain for vel_y",self.configs)
        self._declare_and_fill_map("vel_y_sat", 60, "Saturation for vel_x",self.configs)
        self._declare_and_fill_map("vel_y_n", 52.9421, "Filter_coefficient",self.configs)
        self._declare_and_fill_map("vel_z_p", 618.3667, "Kp gain for vel_z",self.configs)
        self._declare_and_fill_map("vel_z_i", 2061.2148, "Ki gain for vel_z",self.configs)
        self._declare_and_fill_map("vel_z_d", -12.1580, "Kd gain for vel_z",self.configs)
        self._declare_and_fill_map("vel_z_sat", 60, "Saturation for vel_z",self.configs)
        self._declare_and_fill_map("vel_z_n", 15.8085, "Filter_coefficient",self.configs)
        self._declare_and_fill_map("vel_yaw_p", 5.7831, "Kp gain for vel_yaw",self.configs)
        self._declare_and_fill_map("vel_yaw_i", 11.0672, "Ki gain for vel_yaw",self.configs)
        self._declare_and_fill_map("vel_yaw_d", -0.1569, "Kp gain for vel_yaw",self.configs)
        self._declare_and_fill_map("vel_yaw_sat", 10, "Saturation for vel_z",self.configs)
        self._declare_and_fill_map("vel_yaw_n", 30.8659, "Filter_coefficient",self.configs)

        self._declare_and_fill_map(
                    "odom_vel_in_world", True, "Is odometry velocity supplied in world frame? (gazebo)", self.configs)
        self.add_on_set_parameters_callback(self.callback_params)

        self.create_pids(self.configs)

        # ROS infrastructure
        self.sub_cmd_vel = self.create_subscription(geometry_msgs.Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.sub_odometry = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.thrust_alloc_pub = self.create_publisher(geometry_msgs.Wrench, 'thruster_manager/input',10)
        self.body_vel_odom_pub = self.create_publisher(geometry_msgs.TwistStamped, 'body_odom', 10)
        #self.cmd_accel_pub = self.create_publisher(geometry_msgs.Accel, 'cmd_accel',10)

    def _declare_and_fill_map(self, key, value, description, map):
            param = self.declare_parameter(key, value, ParameterDescriptor(description=description))
            map[key] = param.value

    def create_pids(self, configs):
        self.pid_vel_x = PIDRegulator(configs["vel_x_p"], configs["vel_x_i"], configs["vel_x_d"], configs["vel_x_sat"], configs["vel_x_n"])
        self.pid_vel_y = PIDRegulator(configs["vel_y_p"], configs["vel_y_i"], configs["vel_y_d"], configs["vel_y_sat"], configs["vel_y_n"])
        self.pid_vel_z = PIDRegulator(configs["vel_z_p"], configs["vel_z_i"], configs["vel_z_d"], configs["vel_z_sat"], configs["vel_z_n"])
        self.pid_vel_yaw = PIDRegulator(configs["vel_yaw_p"], configs["vel_yaw_i"], configs["vel_yaw_d"], configs["vel_yaw_sat"], configs["vel_yaw_n"])

    def callback_params(self, data):
        """Handling parameter changes"""
        for parameter in data:
            self.configs[parameter.name] = parameter.value

        self.create_pids(self.configs) 
        self.get_logger().warn("Parameter dynamically changed")
        return SetParametersResult(successful=True)

    def cmd_vel_callback(self, msg):
        """Handle updated set velocity callback."""
        # Just store the desired velocity. The actual control runs on odometry callbacks
        v_l = msg.linear
        v_a = msg.angular
        self.v_linear_des = numpy.array([v_l.x, v_l.y, v_l.z])
        self.v_angular_des = numpy.array([v_a.x, v_a.y, v_a.z])

    def odometry_callback(self, msg):
        """Handle updated measured velocity callback."""
        if not bool(self.configs):
            return

        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        v_linear = numpy.array([linear.x, linear.y, linear.z])
        v_angular = numpy.array([angular.x, angular.y, angular.z])

        if self.configs['odom_vel_in_world']:
            # This is a temp. workaround for gazebo's pos3d plugin not behaving properly:
            # Twist should be provided wrt child_frame, gazebo provides it wrt world frame
            # see http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html
            xyzw_array = lambda o: numpy.array([o.x, o.y, o.z, o.w])
            q_wb = xyzw_array(msg.pose.pose.orientation)
            R_bw = transf.quaternion_matrix(q_wb)[0:3, 0:3].transpose()

            
            v_linear = R_bw.dot(v_linear)
            v_angular = R_bw.dot(v_angular)

            body_vel = geometry_msgs.TwistStamped()
            body_vel.header.stamp = msg.header.stamp
            body_vel.header.frame_id = msg.child_frame_id
            body_vel.twist.linear.x = v_linear[0]
            body_vel.twist.linear.y = v_linear[1]
            body_vel.twist.linear.z = v_linear[2]
            body_vel.twist.angular.x = v_angular[0]
            body_vel.twist.angular.y = v_angular[1]
            body_vel.twist.angular.z = v_angular[2]

            self.body_vel_odom_pub.publish(body_vel)

        t = time_in_float_sec_from_msg(msg.header.stamp)
        e_v_linear = (self.v_linear_des - v_linear)
        e_v_angular = (self.v_angular_des - v_angular)

        # a_linear_x = self.pid_vel_x.regulate(e_v_linear[0], t)
        # a_linear_y = self.pid_vel_y.regulate(e_v_linear[1], t)
        # a_linear_z = self.pid_vel_z.regulate(e_v_linear[2], t)

        # a_angular_x = self.pid_vel_roll.regulate(e_v_angular[0], t)
        # a_angular_y = self.pid_vel_pitch.regulate(e_v_angular[1], t)
        # a_angular_z = self.pid_vel_yaw.regulate(e_v_angular[2], t)

        # cmd_accel = geometry_msgs.Accel()
        # cmd_accel.linear = geometry_msgs.Vector3(x=a_linear_x, y=a_linear_y, z=a_linear_z)
        # cmd_accel.angular = geometry_msgs.Vector3(x=a_angular_x, y=a_angular_y, z=a_angular_z)

        # self.cmd_accel_pub.publish(cmd_accel)
        tau_x = self.pid_vel_x.regulate(e_v_linear[0], t)
        tau_y = self.pid_vel_y.regulate(e_v_linear[1], t)
        tau_z = self.pid_vel_z.regulate(e_v_linear[2], t)
        tau_yaw = self.pid_vel_yaw.regulate(e_v_angular[2], t)

        force_msg = geometry_msgs.Wrench()
        force_msg.force.x = tau_x
        force_msg.force.y = tau_y
        force_msg.force.z = tau_z
        force_msg.torque.z = tau_yaw
        self.thrust_alloc_pub.publish(force_msg)

def main():
    print('Starting VelocityControl.py')
    rclpy.init()

    try:
        sim_time_param = is_sim_time()

        node = SingleVelocityControllerNode('velocity_control', parameter_overrides=[sim_time_param])
        rclpy.spin(node)
    except Exception as e:
        print('Caught exception: ' + str(e))
    finally:
        rclpy.shutdown()
    print('Exiting')


#==============================================================================
if __name__ == '__main__':
    main()

