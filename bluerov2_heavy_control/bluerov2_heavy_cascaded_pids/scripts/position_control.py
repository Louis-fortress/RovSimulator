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

class SinglePositionControllerNode(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        self.pos_des = numpy.zeros(3)
        self.quat_des = numpy.array([0, 0, 0, 1])

        self.pid_configs = {}

        self.initialized = False
        self.have_command = False

        #ROS infrastructure
        self.cmd_pose_sub = self.create_subscription(geometry_msgs.PoseStamped, 'cmd_pose', self.cmd_pose_callback, 10)
        self.odometry_sub = self.create_subscription(Odometry, 'pose_gt', self.odometry_callback, 10)
        self.thrust_alloc_pub = self.create_publisher(geometry_msgs.Wrench, 'thruster_manager/input',10)

        self.pid_pos_x = PIDRegulator(1, 0, 0, 10, 1)
        self.pid_pos_y = PIDRegulator(1, 0, 0, 10, 1)
        self.pid_pos_z = PIDRegulator(1, 0, 0, 10, 1)
        self.pid_rot_roll = PIDRegulator(1, 0, 0, 10, 1)
        self.pid_rot_pitch = PIDRegulator(1, 0, 0, 10, 1)
        self.pid_pos_yaw = PIDRegulator(1, 0, 0, 10, 1)
    
        self._declare_and_fill_map("pos_x_p", 254.9766, "Kp gain on the x", self.pid_configs)
        self._declare_and_fill_map("pos_x_i", 109.8466, "Ki gain on the x", self.pid_configs)
        self._declare_and_fill_map("pos_x_d", 127.9453, "Kd gain on the x", self.pid_configs)
        self._declare_and_fill_map("pos_x_sat", 97.066, "saturation on the x", self.pid_configs)
        self._declare_and_fill_map("pos_x_n", 158.3411, "filter_coefficient", self.pid_configs)
        self._declare_and_fill_map("pos_y_p", 84.9394, "Kp gain on the y", self.pid_configs)
        self._declare_and_fill_map("pos_y_i", 18.6462, "Ki gain on the y", self.pid_configs)
        self._declare_and_fill_map("pos_y_d", 96.3076, "Kd gain on the y", self.pid_configs)
        self._declare_and_fill_map("pos_y_sat", 97.066, "saturation on the y", self.pid_configs)
        self._declare_and_fill_map("pos_y_n", 146.4594, "filter_coefficient", self.pid_configs)
        self._declare_and_fill_map("pos_z_p", 679.0364, "Kp gain on the z", self.pid_configs)
        self._declare_and_fill_map("pos_z_i", 372.5506, "Ki gain on the z", self.pid_configs)
        self._declare_and_fill_map("pos_z_d", 271.3377, "Kd gain on the z", self.pid_configs)
        self._declare_and_fill_map("pos_z_sat", 137.29, "saturation on the z", self.pid_configs)
        self._declare_and_fill_map("pos_z_n", 166.1685, "filter_coefficient", self.pid_configs)
        self._declare_and_fill_map("rot_roll_p", 147.3812, "Kp gain on the roll", self.pid_configs)
        self._declare_and_fill_map("rot_roll_i", 91.7870, "Ki gain on the roll", self.pid_configs)
        self._declare_and_fill_map("rot_roll_d", 35.5611, "Kd gain on the roll", self.pid_configs)
        self._declare_and_fill_map("rot_roll_sat", 38.167, "saturation on the roll", self.pid_configs)
        self._declare_and_fill_map("rot_roll_n", 196.0862, "filter_coefficient", self.pid_configs)
        self._declare_and_fill_map("rot_pitch_p", 62.3204, "Kp gain on the pitch", self.pid_configs)
        self._declare_and_fill_map("rot_pitch_i", 84.9584, "Ki gain on the pitch", self.pid_configs)
        self._declare_and_fill_map("rot_pitch_d", 11.0785, "Kd gain on the pitch", self.pid_configs)
        self._declare_and_fill_map("rot_pitch_sat", 5.0, "saturation on the pitch", self.pid_configs)
        self._declare_and_fill_map("rot_pitch_n", 124.8518, "filter_coefficient", self.pid_configs)
        self._declare_and_fill_map("rot_yaw_p", 6.0982, "Kp gain on the yaw", self.pid_configs)
        self._declare_and_fill_map("rot_yaw_i", 1.9403, "Ki gain on the yaw", self.pid_configs)
        self._declare_and_fill_map("rot_yaw_d", 4.6137, "Kd gain on the yaw", self.pid_configs)
        self._declare_and_fill_map("rot_yaw_sat", 5.0, "saturation on the yaw", self.pid_configs)
        self._declare_and_fill_map("rot_yaw_n", 58.8401, "filter_coefficient", self.pid_configs)

        self.add_on_set_parameters_callback(self.callback_params)

        self.create_pids(self.pid_configs)

    def _declare_and_fill_map(self, key, value, description, map):
        param = self.declare_parameter(key, value, ParameterDescriptor(description=description))
        map[key] = param.value

    def create_pids(self, config):
        self.pid_pos_x = PIDRegulator(config["pos_x_p"], config["pos_x_i"], config["pos_x_d"], config["pos_x_sat"], config["pos_x_n"])
        self.pid_pos_y = PIDRegulator(config["pos_y_p"], config["pos_y_i"], config["pos_y_d"], config["pos_y_sat"], config["pos_y_n"])
        self.pid_pos_z = PIDRegulator(config["pos_z_p"], config["pos_z_i"], config["pos_z_d"], config["pos_z_sat"], config["pos_z_n"])
        self.pid_rot_roll = PIDRegulator(config["rot_roll_p"], config["rot_roll_i"], config["rot_roll_d"], config["rot_roll_sat"], config["rot_roll_n"])
        self.pid_rot_pitch = PIDRegulator(config["rot_pitch_p"], config["rot_pitch_i"], config["rot_pitch_d"], config["rot_pitch_sat"], config["rot_pitch_n"])
        self.pid_rot_yaw = PIDRegulator(config["rot_yaw_p"], config["rot_yaw_i"], config["rot_yaw_d"], config["rot_yaw_sat"], config["rot_yaw_n"])
        
    def callback_params(self, data):
        """Handling parameter changes"""
        for parameter in data:
            self.pid_configs[parameter.name] = parameter.value

        self.create_pids(self.pid_configs) 
        self.get_logger().warn("Parameter dynamically changed")
        return SetParametersResult(successful=True)

    
    def cmd_pose_callback(self, msg):
        """Handling updated set pose callback"""
        #Storing the desired pose
        p = msg.pose.position
        q = msg.pose.orientation
        self.pos_des = numpy.array([p.x, p.y, p.z])
        self.quat_des = numpy.array([q.x, q.y, q.z, q.w])
        self.have_command = True

    def odometry_callback(self, msg):
        """Handle odom callback"""
        if not bool(self.pid_configs):
            return

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        p = numpy.array([p.x, p.y, p.z])
        q = numpy.array([q.x, q.y, q.z,q.w])
        

        if not self.initialized:
            #If this is the first callback store and hold the latest pose
            if not self.have_command:
                self.pos_des = p
                self.quat_des = q
            self.initialized = True

        t = time_in_float_sec_from_msg(msg.header.stamp)

        #Position error
        e_pos_world = self.pos_des - p
        R = transf.quaternion_matrix(q)[0:3, 0:3]
        e_pos_body = R.T.dot(e_pos_world)

        #Error quaternion wrt body frame
        e_rot_quat = transf.quaternion_multiply(transf.quaternion_conjugate(q), self.quat_des)

        #Error angles
        e_rot = numpy.array(transf.euler_from_quaternion(e_rot_quat))

        #Angle wrapping in the case of larger angles
        e_rot = numpy.array([
            numpy.arctan2(numpy.sin(e_rot[0]), numpy.cos(e_rot[0])),
            numpy.arctan2(numpy.sin(e_rot[1]), numpy.cos(e_rot[1])),
            numpy.arctan2(numpy.sin(e_rot[2]), numpy.cos(e_rot[2]))
        ])

        #Implementatoin of Single PIDs
        tau_x = self.pid_pos_x.regulate(e_pos_body[0], t)
        tau_y = self.pid_pos_y.regulate(e_pos_body[1], t)
        tau_z = self.pid_pos_z.regulate(e_pos_body[2], t)
        tau_k = self.pid_rot_roll.regulate(e_rot[0], t)
        tau_m = self.pid_rot_pitch.regulate(e_rot[1], t)
        tau_n = self.pid_rot_yaw.regulate(e_rot[2], t)

        force_msg = geometry_msgs.Wrench()
        force_msg.force.x = tau_x
        force_msg.force.y = tau_y
        force_msg.force.z = tau_z
        force_msg.torque.x = tau_k
        force_msg.torque.y = tau_m
        force_msg.torque.z = tau_n
        self.thrust_alloc_pub.publish(force_msg)


def main():
    print('Starting VelocityControl.py')
    rclpy.init()

    try:
        sim_time_param = is_sim_time()

        node = SinglePositionControllerNode('velocity_control', parameter_overrides=[sim_time_param])
        rclpy.spin(node)
    except Exception as e:
        print('Caught exception: ' + str(e))
    finally:
        rclpy.shutdown()
    print('Exiting')

if __name__ == '__main__':
    main()