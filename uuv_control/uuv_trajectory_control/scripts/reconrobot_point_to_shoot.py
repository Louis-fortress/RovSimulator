#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
#from uuv_assistants.src.tf_quaternion.transformations import euler_from_quaternion
import numpy as np

KP_DEPTH        = 40.0
KI_DEPTH        = 0.5
KD_DEPTH        = 10.0
 
KP_YAW          = 30.0
KI_YAW          = 0.1
KD_YAW          = 8.0
 
# Lighter depth correction while surging forward (less aggressive to avoid conflict)
KP_DEPTH_DRIVE  = 30.0
KI_DEPTH_DRIVE  = 0.3
KD_DEPTH_DRIVE  = 6.0
 
# ── Control parameters ─────────────────────────────────────────────────────────
SURGE_FORCE     = 50.0   # Newtons of forward thrust in DRIVE state
MAX_FORCE       = 200.0  # Hard clamp on all wrench outputs (N or N·m)
 
# State transition thresholds
DEPTH_THRESH    = 0.05   # metres  — exit DEPTH_HOLD when |z_error| < this
YAW_THRESH      = 0.05   # radians (~3°) — exit YAW_TO_GOAL when |yaw_err| < this
ARRIVAL_THRESH  = 1.0    # metres  — exit DRIVE when XY distance < this
NAMESPACE = 'reconrobot'
RATE_HZ = 20

def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))

def angle_wrap(a: float):
    """Wrap an angle between the range [-pi,pi]"""
    while a > math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a

class PID:
    """Simple PID controller with anti-windup clamp."""
 
    def __init__(self, kp: float, ki: float, kd: float, limit: float = MAX_FORCE):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self._integral   = 0.0
        self._prev_error = None
 
    def reset(self):
        self._integral   = 0.0
        self._prev_error = None
 
    def compute(self, error: float, dt: float) -> float:
        # Integrate with anti-windup (clamp integral contribution)
        self._integral += error * dt
        integral_term = clamp(self.ki * self._integral, self.limit)
 
        derivative = 0.0
        if self._prev_error is not None:
            derivative = (error - self._prev_error) / max(dt, 1e-6)
        self._prev_error = error
 
        output = self.kp * error + integral_term + self.kd * derivative
        return clamp(output, self.limit)


class PointToShootController(Node):
    """Using Finite state machine for this controller"""
    #State constants
    IDLE = 'IDLE'
    DEPTH_HOLD = 'DEPTH_HOLD'
    YAW_TO_GOAL = 'YAW_TO_GOAL'
    DRIVE = 'DRIVE'
    _LABEL = 'PID'

    def __init__(self):
        super().__init__('point_to_shoot_controller')
        self._tau = np.zeros(6)
        self._is_init = True
        ns = NAMESPACE
        #Current vehicle pose
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.pose_received = False

        #Goal
        self.goal = None
        self.state = self.IDLE

        #PID controllers
        self.pid_depth       = PID(KP_DEPTH,       KI_DEPTH,       KD_DEPTH)
        self.pid_yaw         = PID(KP_YAW,         KI_YAW,         KD_YAW)
        self.pid_depth_drive = PID(KP_DEPTH_DRIVE, KI_DEPTH_DRIVE, KD_DEPTH_DRIVE)

    
        #Subscriptions
        self.create_subscription(Odometry, 
                                 f'/{ns}/pose_gt',
                                 self.pose_callback, 10)
        self.create_subscription(Point, f'/{ns}/goto_target',
                                 self.goal_callback, 10)
        
        #Publisher
        self._wrench_pub = self.create_publisher(WrenchStamped, f'/{ns}/thruster_manager/input_stamped',10)

        self.last_time = self.get_clock().now()
        self.log_tick = 0
        self.create_timer(1.0/RATE_HZ, self.control_loop)

    def pose_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.x = pos.x
        self.y = pos.y
        self.z = pos.z
        q = [ori.x, ori.y, ori.z, ori.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(q)
        self.pose_received = True

    def goal_callback(self, msg):
        self.goal = msg
        self.state = self.DEPTH_HOLD

        #Reset all pids when a new goal is received
        self.pid_depth.reset()
        self.pid_yaw.reset()
        self.pid_depth_drive.reset()

    def control_loop(self):
        if not self.pose_received or self.state==self.IDLE:
            return
        
        #compute dt
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1.e-9
        self.last_time = now
        if dt <= 0.0:
            return
        g = self.goal

        depth_error = g.z - self.z

        #bearing to goal in world frame
        bearing = math.atan2(g.y - self.y, g.x - self.x)
        yaw_error = angle_wrap(bearing - self.yaw)

        #Horizontal distance
        xy_dist = math.hypot(g.x - self.x, g.y - self.y)

        #Wrench forces in body frame
        fx = fy = fz = tx = ty = tz = 0.0

        if self.state == self.DEPTH_HOLD:
            fz = self.pid_depth.compute(depth_error, dt)

            if abs(depth_error) < DEPTH_THRESH:
                self.state = self.YAW_TO_GOAL
                self.pid_depth.reset()
                self.pid_yaw.reset()
                self.get_logger().info(
                    f'Depth achieved (z={self.z:.2f} m, err={depth_error:.3f} m) '
                    f'— entering YAW_TO_GOAL'
                )
                
                

        elif self.state == self.YAW_TO_GOAL:
            tz = self.pid_yaw.compute(yaw_error, dt)
            fz = self.pid_depth.compute(depth_error, dt)

            if abs(yaw_error) < YAW_THRESH:
                self.state = self.DRIVE
                self.pid_yaw.reset()
                self.pid_depth_drive.reset()
                self.last_time = self.get_clock().now()  #reset dt clock on transition
                self.get_logger().info(
                    f'Aligned with goal (yaw={math.degrees(self.yaw):.1f} degrees, err={math.degrees(yaw_error):.3f} degrees) '
                    f'— entering YAW_TO_GOAL'
                )
                

        elif self.state == self.DRIVE:
            fx = SURGE_FORCE
            tz = self.pid_yaw.compute(yaw_error, dt) #heading hold
            fz = self.pid_depth_drive.compute(depth_error, dt) #depth hold

            if xy_dist < ARRIVAL_THRESH:
                self.state = self.DEPTH_HOLD
                self.get_logger().info(
                    f'Arrived at goal (xy_dist={xy_dist:.2f} m) — IDLE'
                )
                self._publish_wrench(0.0, 0.0, 0.0, 0.0, 0.0, 0)
                return
            
        # ── Publish wrench ────────────────────────────────────────────────────
        self._publish_wrench(fx, fy, fz, tx, ty, tz)

        # ── Wrench publisher ──────────────────────────────────────────────────────
 
    def _publish_wrench(self, fx: float, fy: float, fz: float,
                        tx: float, ty: float, tz: float):
        msg = WrenchStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{NAMESPACE}/base_link'
        msg.wrench.force.x  = float(clamp(fx, MAX_FORCE))
        msg.wrench.force.y  = float(clamp(fy, MAX_FORCE))
        msg.wrench.force.z  = float(clamp(fz, MAX_FORCE))
        msg.wrench.torque.x = float(clamp(tx, MAX_FORCE))
        msg.wrench.torque.y = float(clamp(ty, MAX_FORCE))
        msg.wrench.torque.z = float(clamp(tz, MAX_FORCE))
        self._wrench_pub.publish(msg)

def main():
    rclpy.init()
    node = PointToShootController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
