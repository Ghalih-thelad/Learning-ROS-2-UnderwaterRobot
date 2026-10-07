#!/usr/bin/env python3

import math

from matplotlib.pylab import angle
from std_msgs.msg import String

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class ThrusterController(Node):

    def __init__(self):
        super().__init__('thruster_controller')
        self.control_mode = 'AUV'

        self.mode_sub = self.create_subscription(
           String,
           '/auv/control_mode',
            self.mode_callback,
            10
        )   

        # =====================================================
        # Parameters
        # =====================================================

        self.declare_parameter('max_thrust', 15.0)
        self.declare_parameter('max_yaw_thrust', 2.0)
        self.declare_parameter('yaw_kp', 14.0)
        self.declare_parameter('yaw_ki', 0.2)
        self.declare_parameter('speed_kp', 0.4)
        self.declare_parameter('speed_ki', 0.1)
        self.declare_parameter('max_target_speed', 1.5)

        # Maximum elevator deflection
        self.declare_parameter('max_fin_angle', 0.4)

        # Maximum desired pitch angle
        self.declare_parameter(
            'max_pitch_angle',
            math.radians(20.0)
        )
        self.declare_parameter('heading_kp', 1.0)
        self.declare_parameter('max_heading_yaw_rate', 0.4)

        self.current_depth = 0.0
        self.desired_depth = self.current_depth
        self.depth_control_enabled = False
        self.depth_kp = 0.15

        self.depth_target_sub = self.create_subscription(
            Float64,
            '/auv/target_depth',
            self.depth_target_callback,
            10
        )

        #odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/auv/odom',
            self.odom_callback,
            10
        )

        # Initial PID gains
        self.declare_parameter('pitch_kp', 0.5)
        self.declare_parameter('pitch_ki', 0.0)
        self.declare_parameter('pitch_kd', 0.9)

        self.max_thrust = self.get_parameter(
            'max_thrust'
        ).value

        self.max_yaw_thrust = self.get_parameter(
            'max_yaw_thrust'
        ).value

        self.yaw_kp = self.get_parameter(
            'yaw_kp'
        ).value

        self.yaw_ki = self.get_parameter(
            'yaw_ki'
        ).value

        self.heading_kp = self.get_parameter(
            'heading_kp'
        ).value

        self.max_heading_yaw_rate = self.get_parameter(
            'max_heading_yaw_rate'
        ).value

        self.speed_kp = self.get_parameter(
            'speed_kp'
        ).value

        self.speed_ki = self.get_parameter(
            'speed_ki'
        ).value

        self.max_target_speed = self.get_parameter(
            'max_target_speed'
        ).value

        self.max_fin_angle = self.get_parameter(
            'max_fin_angle'
        ).value

        self.max_pitch_angle = self.get_parameter(
            'max_pitch_angle'
        ).value

        self.pitch_kp = self.get_parameter(
            'pitch_kp'
        ).value

        self.pitch_ki = self.get_parameter(
            'pitch_ki'
        ).value

        self.pitch_kd = self.get_parameter(
            'pitch_kd'
        ).value

        # =====================================================
        # Desired states
        # =====================================================

        self.desired_pitch = 0.0
        self.pitch_integral = 0.0
        self.desired_forward = 0.0
        self.desired_yaw_rate = 0.0
        self.current_yaw_rate = 0.0
        self.current_heading = 0.0
        self.desired_heading = 0.0
        self.heading_control_enabled = False
        self.yaw_integral = 0.0
        self.last_odom_time = None
        self.current_speed = 0.0
        self.desired_speed = 0.0
        self.speed_integral = 0.0
        self.speed_control_enabled = False

        # =====================================================
        # Thruster publishers
        # =====================================================

        self.left_pub = self.create_publisher(
            Float64,
            '/model/auv/joint/left_propeller_joint/cmd_thrust',
            10
        )

        self.right_pub = self.create_publisher(
            Float64,
            '/model/auv/joint/right_propeller_joint/cmd_thrust',
            10
        )

        # =====================================================
        # Fin publishers
        # =====================================================

        self.left_fin_pub = self.create_publisher(
            Float64MultiArray,
            '/left_fin_controller/commands',
            10
        )

        self.right_fin_pub = self.create_publisher(
            Float64MultiArray,
            '/right_fin_controller/commands',
            10
        )

        # =====================================================
        # Subscribers
        # =====================================================

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/auv/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        self.heading_target_sub = self.create_subscription(
            Float64,
            '/auv/target_heading',
            self.heading_target_callback,
            10
        )

        self.speed_target_sub = self.create_subscription(
            Float64,
            '/auv/target_speed',
            self.speed_target_callback,
            10
        )

        self.get_logger().info(
            'AUV controller with pitch PID started'
        )

    # =========================================================
    # Utility
    # =========================================================

    def clamp(self, value, minimum, maximum):
        return max(minimum, min(value, maximum))

    def wrap_angle(self, angle):
        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    def mode_callback(self, msg):

        new_mode = msg.data.upper()

        if new_mode not in ['ROV', 'AUV']:
            self.get_logger().warn(
                f'Invalid control mode: {msg.data}'
            )
            return

        if new_mode == self.control_mode:
            return

        self.control_mode = new_mode

        # Reset controller memory when changing mode
        self.pitch_integral = 0.0
        self.yaw_integral = 0.0

        if self.control_mode == 'ROV':

            self.depth_control_enabled = False
            self.heading_control_enabled = False
            self.speed_control_enabled = False

            self.desired_pitch = 0.0
            self.desired_yaw_rate = 0.0
            self.desired_forward = 0.0

        elif self.control_mode == 'AUV':

            self.desired_depth = self.current_depth
            self.depth_control_enabled = True

            self.desired_heading = self.current_heading
            self.heading_control_enabled = True

            self.desired_speed = self.current_speed
            self.speed_control_enabled = True

        self.get_logger().info(
            f'Control mode changed to {self.control_mode}'
        )
    
    # =========================================================
    # cmd_vel callback
    # =========================================================

    def cmd_vel_callback(self, msg):

        if self.control_mode == 'ROV':

            self.desired_forward = self.clamp(
                msg.linear.x,
                -1.0,
                1.0
            )

            new_yaw_rate = self.clamp(
                msg.angular.z,
                -1.0,
                1.0
            )

            if abs(
                new_yaw_rate -
                self.desired_yaw_rate
            ) > 1e-6:
                self.yaw_integral = 0.0

            self.desired_yaw_rate = new_yaw_rate

            pitch_command = self.clamp(
                msg.angular.y,
                -1.0,
                1.0
            )

            self.desired_pitch = (
                -pitch_command *
                self.max_pitch_angle
            )

        elif self.control_mode == 'AUV':

            # desired_pitch comes from depth controller
            pass

        self.get_logger().info(
            f'Forward: {self.desired_forward:.2f} | '
            f'Desired yaw rate: {self.desired_yaw_rate:.2f} rad/s | '
            f'Desired pitch: '
            f'{math.degrees(self.desired_pitch):.1f} deg'
        )

    def speed_target_callback(self, msg):

        if self.control_mode != 'AUV':
            self.get_logger().warn(
                'Ignoring speed target because controller is in ROV mode'
            )
            return

        new_speed = self.clamp(
            msg.data,
            -self.max_target_speed,
            self.max_target_speed
        )

        if abs(new_speed - self.desired_speed) > 1e-6:
            self.speed_integral = 0.0

        self.desired_speed = new_speed
        self.speed_control_enabled = True

        self.get_logger().info(
            f'New target speed: {self.desired_speed:.2f} m/s'
        )

    def heading_target_callback(self, msg):

        if self.control_mode != 'AUV':
            self.get_logger().warn(
                'Ignoring heading target because controller is in ROV mode'
            )
            return

        self.desired_heading = self.wrap_angle(
            msg.data
        )

        self.heading_control_enabled = True

        # New maneuver, clear old yaw integral memory
        self.yaw_integral = 0.0

        self.get_logger().info(
            f'New target heading: '
            f'{math.degrees(self.desired_heading):.1f} deg'
        )

    def update_thrusters(self, dt):

        forward_thrust = (
            self.desired_forward *
            self.max_thrust
        )

        yaw_error = (
            self.desired_yaw_rate -
            self.current_yaw_rate
        )

        if dt > 0.0:
            self.yaw_integral += yaw_error * dt

        # Anti-windup
        self.yaw_integral = self.clamp(
            self.yaw_integral,
            -4.0,
            4.0
        )

        yaw_thrust = (
            self.yaw_kp * yaw_error
            +
            self.yaw_ki * self.yaw_integral
        )

        yaw_thrust = self.clamp(
            yaw_thrust,
            -self.max_yaw_thrust,
            self.max_yaw_thrust
        )

        left_thrust = (
            forward_thrust -
            yaw_thrust
        )

        right_thrust = (
            forward_thrust +
            yaw_thrust
        )

        left_thrust = self.clamp(
            left_thrust,
            -self.max_thrust,
            self.max_thrust
        )

        right_thrust = self.clamp(
            right_thrust,
            -self.max_thrust,
            self.max_thrust
        )

        left_msg = Float64()
        right_msg = Float64()

        left_msg.data = left_thrust
        right_msg.data = right_thrust

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
    
    # =========================================================
    # IMU callback
    # =========================================================

    def imu_callback(self, msg):

        q = msg.orientation

        # -----------------------------------------------------
        # Quaternion -> pitch
        # -----------------------------------------------------

        sin_pitch = 2.0 * (
            q.w * q.y -
            q.z * q.x
        )

        sin_pitch = self.clamp(
            sin_pitch,
            -1.0,
            1.0
        )

        current_pitch = math.asin(
            sin_pitch
        )

        # -----------------------------------------------------
        # Pitch error
        # -----------------------------------------------------

        error = (
            self.desired_pitch -
            current_pitch
        )

        # -----------------------------------------------------
        # Integral
        # -----------------------------------------------------

        # IMU is running at approximately 100 Hz
        dt = 0.01

        self.pitch_integral += (
            error * dt
        )

        # Prevent integral windup
        self.pitch_integral = self.clamp(
            self.pitch_integral,
            -0.5,
            0.5
        )

        # -----------------------------------------------------
        # Derivative
        # -----------------------------------------------------

        # IMU already gives pitch angular velocity
        pitch_rate = msg.angular_velocity.y

        # -----------------------------------------------------
        # PID
        # -----------------------------------------------------

        pitch_control = (
            self.pitch_kp * error
            +
            self.pitch_ki * self.pitch_integral
            -
            self.pitch_kd * pitch_rate
        )

        # -----------------------------------------------------
        # Fin command
        # -----------------------------------------------------

        # IMPORTANT:
        # In your model:
        #
        # negative fin -> positive body pitch
        # positive fin -> negative body pitch
        #
        # So PID output must be inverted.
        
        fin_angle = -pitch_control

        fin_angle = self.clamp(
            fin_angle,
            -self.max_fin_angle,
            self.max_fin_angle
       )

        left_fin_msg = Float64MultiArray()
        right_fin_msg = Float64MultiArray()

        left_fin_msg.data = [fin_angle]
        right_fin_msg.data = [fin_angle]

        self.left_fin_pub.publish(
            left_fin_msg
        )

        self.right_fin_pub.publish(
            right_fin_msg
        )

        self.get_logger().info(
            f'Pitch desired: {math.degrees(self.desired_pitch):.1f} deg | '
            f'actual: {math.degrees(current_pitch):.1f} deg | '
            f'error: {math.degrees(error):.1f} deg | '
            f'pitch rate: {math.degrees(pitch_rate):.1f} deg/s | '
            f'fin: {fin_angle:.3f} rad'
        )

    # =========================================================
    # Odometry callback
    # =========================================================

    def depth_target_callback(self, msg):
        
        if self.control_mode != 'AUV':
            self.get_logger().warn(
                'Ignoring depth target because controller is in ROV mode'
            )
            return
        self.desired_depth = max(0.0, msg.data)
        self.depth_control_enabled = True

        self.get_logger().info(
            f'New target depth: {self.desired_depth:.2f} m'
        )

    def odom_callback(self, msg):
        self.current_depth = -msg.pose.pose.position.z

        q = msg.pose.pose.orientation

        sin_yaw = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cos_yaw = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        self.current_heading = math.atan2(
            sin_yaw,
            cos_yaw
        )

         # Current yaw rate from odometry
        self.current_yaw_rate = (
            msg.twist.twist.angular.z
        )

        self.current_speed = (
            msg.twist.twist.linear.x
        )

        current_time = (
            msg.header.stamp.sec +
            msg.header.stamp.nanosec * 1e-9
        )

        if self.last_odom_time is None:
            dt = 0.0
        else:
            dt = current_time - self.last_odom_time

            # Avoid weird jumps
            dt = self.clamp(
                dt,
                0.0,
                0.1
            )

        self.last_odom_time = current_time

        if (
            self.control_mode == 'AUV'
            and self.heading_control_enabled
        ):

            heading_error = self.wrap_angle(
                self.desired_heading -
                self.current_heading
            )

            self.desired_yaw_rate = self.clamp(
                self.heading_kp * heading_error,
                -self.max_heading_yaw_rate,
                self.max_heading_yaw_rate
            )

        if (
            self.control_mode == 'AUV'
            and self.speed_control_enabled
        ):

            speed_error = (
                self.desired_speed -
                self.current_speed
            )

            if dt > 0.0:
                self.speed_integral += (
                    speed_error * dt
                )

            self.speed_integral = self.clamp(
                self.speed_integral,
                -2.0,
                2.0
            )

            speed_command = (
                self.speed_kp * speed_error
                +
                self.speed_ki * self.speed_integral
            )

            self.desired_forward = self.clamp(
                speed_command,
                -1.0,
                1.0
            )

        # Closed-loop thruster control
        self.update_thrusters(dt)

        if (
            self.control_mode != 'AUV'
            or not self.depth_control_enabled
        ):
            return
        
        depth_error = self.desired_depth - self.current_depth

        # Positive depth error = need to go deeper
        desired_pitch = self.depth_kp * depth_error

        # Limit depth controller to ±10 degrees pitch
        max_depth_pitch = math.radians(20.0)

        self.desired_pitch = self.clamp(
            desired_pitch,
            -max_depth_pitch,
            max_depth_pitch
        )

        self.get_logger().info(
            f'Depth desired: {self.desired_depth:.2f} m | '
            f'actual: {self.current_depth:.2f} m | '
            f'pitch target: {math.degrees(self.desired_pitch):.1f} deg'
        )


def main(args=None):

    rclpy.init(args=args)

    node = ThrusterController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()