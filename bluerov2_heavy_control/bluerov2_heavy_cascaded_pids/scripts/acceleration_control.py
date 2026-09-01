#!/usr/bin/env python3
import numpy
import rclpy

from geometry_msgs.msg import Accel
from geometry_msgs.msg import Wrench
from rclpy.node import Node

from plankton_utils.time import is_sim_time


class AccelerationControllerNode(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name,
                        allow_undeclared_parameters=True, 
                        automatically_declare_parameters_from_overrides=True,
                          **kwargs)

        self.ready = False
        self.mass = 1.
        self.inertial_tensor = numpy.identity(3)    
        self.mass_inertial_matrix = numpy.zeros((6,6))
      
        #ROS infrastructure
        self.sub_accel = self.create_subscription(Accel, 'cmd_accel', self.accel_callback, 10)
        self.pub_wrench = self.create_publisher(Wrench, 'thruster_manager/input', 10)

        self.get_logger().info(str(self.get_parameters(['/'])))

        if not self.has_parameter("pid.mass"):
            raise RuntimeError("UUV's mass was not provided")

        self.mass = self.get_parameter("pid.mass").value
        self.inertial = self.get_parameters_by_prefix("pid.inertial")

        if len(self.inertial) == 0:
            raise RuntimeError("UUV Inertial not provided")

        self.inertial_tensor = numpy.array(
            [[self.inertial['ixx'].value, self.inertial['ixy'].value, self.inertial['ixz'].value],
             [self.inertial['ixy'].value, self.inertial['iyy'].value, self.inertial['iyz'].value],
             [self.inertial['ixz'].value, self.inertial['iyz'].value, self.inertial['izz'].value]]
        )
        self.mass_inertial_matrix = numpy.vstack((
            numpy.hstack((self.mass*numpy.identity(3), numpy.zeros((3, 3)))),
            numpy.hstack((numpy.zeros((3,3)), self.inertial_tensor))
        ))

        self.get_logger().info(str(self.mass_inertial_matrix))
        self.ready = True


    def accel_callback(self, msg):
        if not self.ready:
            return

        #Extracting acceleration from the velocity controller
        linear = numpy.array((msg.linear.x, msg.linear.y, msg.linear.z))
        angular = numpy.array((msg.angular.x, msg.angular.y, msg.angular.z))
        accel = numpy.hstack((linear, angular)).transpose()

        #Convert acceleration to force / torque
        force_torque = self.mass_inertial_matrix.dot(accel)

        force_msg = Wrench()
        force_msg.force.x = force_torque[0]
        force_msg.force.y = force_torque[1]
        force_msg.force.z = force_torque[2]

        force_msg.torque.x = force_torque[3]
        force_msg.torque.y = force_torque[4]
        force_msg.torque.z = force_torque[5]

        self.pub_wrench.publish(force_msg)


def main():
    print('starting acceleration_control.py')

    rclpy.init()

    try:
        sim_time_param = is_sim_time()

        node = AccelerationControllerNode('acceleration_control', parameter_overrides=[sim_time_param])
        rclpy.spin(node)
    except Exception as e:
        print('Caught exception:' + str(e))
    finally:
        if rclpy.ok():
            rclpy.shutdown()
            print('Exiting')

#==============================================================================
if __name__ == '__main__':
    main()