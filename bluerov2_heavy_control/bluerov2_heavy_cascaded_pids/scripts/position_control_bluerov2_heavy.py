#!/usr/bin/env python3
import numpy
import rclpy
import tf_quaternion.transformations as transf
import geometry_msgs.msg as geometry_msgs
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node


from plankton_utils.time import time_in_float_sec_from_msg
from plankton_utils.time import is_sim_time
from bluerov2_heavy_PID.PIDRegulator import PIDRegulator
#from uuv_PID import PIDRegulator

class RollPitchAttitudeControllerNode(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        self.quat_des = numpy.array([0.0, 0.0, 0.0, 1])

        self.pid_configs = {}

        self.initialized = False
        self.have_command = False

        #ROS infrastructure
        self.odometry_sub = self.create_subscription(Odometry, 'pose_gt', self.odometry_callback, 10)
        self.thrust_alloc_pub = self.create_publisher(geometry_msgs.Wrench, 'thruster_manager/input',10)
        self.cmd_vel_pub = self.create_publisher(geometry_msgs.Twist, 'cmd_attitude_vel', 10) 


        self.pid_rot_yaw = PIDRegulator(1, 0, 0, 1.5, 0)
        
        
        self._declare_and_fill_map("rot_yaw_p", 2, "Kd gain on the yaw", self.pid_configs)
        self._declare_and_fill_map("rot_yaw_sat", 1.0, "saturation on the yaw", self.pid_configs)     

        self.add_on_set_parameters_callback(self.callback_params)

        self.create_pids(self.pid_configs)

    def _declare_and_fill_map(self, key, value, description, map):
        param = self.declare_parameter(key, value, ParameterDescriptor(description=description))
        map[key] = param.value

    def create_pids(self, config):
        self.pid_rot_yaw = PIDRegulator(config["rot_yaw_p"], 0, 0, config["rot_yaw_sat"], 0)
        
        
    def callback_params(self, data):
        """Handling parameter changes"""
        for parameter in data:
            self.pid_configs[parameter.name] = parameter.value

        self.create_pids(self.pid_configs) 
        self.get_logger().warn("Parameter dynamically changed")
        return SetParametersResult(successful=True)


    def odometry_callback(self, msg):
        """Handle odom callback"""
        if not bool(self.pid_configs):
            return

        q = msg.pose.pose.orientation
        q = numpy.array([q.x, q.y, q.z,q.w])
        

        t = time_in_float_sec_from_msg(msg.header.stamp)

        #Error quaternion wrt body frame
        e_rot_quat = transf.quaternion_multiply(transf.quaternion_conjugate(q), self.quat_des)

        #Error angles
        e_rot = numpy.array(transf.euler_from_quaternion(e_rot_quat))

        yaw_error = numpy.arctan2(
            numpy.sin(e_rot[2]),
            numpy.cos(e_rot[2])
        )

        yaw_rate_des = self.pid_rot_yaw.regulate(
            yaw_error,
            t
        )

        vel_msg = geometry_msgs.Twist()
        vel_msg.angular.z = yaw_rate_des
        self.cmd_vel_pub.publish(vel_msg)


def main():
    print('Starting RollPitchAttitudeControllerNode')
    rclpy.init()

    try:
        sim_time_param = is_sim_time()

        node = RollPitchAttitudeControllerNode('position_control_bluerov2_heavy', parameter_overrides=[sim_time_param])
        rclpy.spin(node)
    except Exception as e:
        print('Caught exception: ' + str(e))
    finally:
        rclpy.shutdown()
    print('Exiting')

if __name__ == '__main__':
    main()