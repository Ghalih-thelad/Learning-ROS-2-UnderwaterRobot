#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from std_msgs.msg import Float64
from std_msgs.msg import String


class GamepadController(Node):

    def __init__(self):

        super().__init__('gamepad_controller')

        # =====================================================
        # Gamepad mapping
        # =====================================================

        # Confirmed mapping:
        #
        # axes[0] = Left Stick X
        # axes[1] = Left Stick Y
        # axes[4] = Right Stick Y
        #
        # buttons[0] = A

        self.AXIS_YAW = 0
        self.AXIS_FORWARD = 1
        self.AXIS_PITCH = 4

        self.BUTTON_MODE = 0

        # =====================================================
        # Parameters
        # =====================================================

        self.declare_parameter(
            'deadzone',
            0.08
        )

        # ROV mode
        self.declare_parameter(
            'max_rov_yaw_rate',
            1.0
        )

        # AUV mode
        self.declare_parameter(
            'max_auv_speed',
            1.5
        )

        # How quickly the heading target moves
        # when Left Stick X is held
        self.declare_parameter(
            'heading_adjust_rate',
            0.8
        )

        # How quickly the depth target moves
        # when Right Stick Y is held
        self.declare_parameter(
            'depth_adjust_rate',
            0.5
        )

        self.declare_parameter(
            'max_depth',
            20.0
        )

        self.declare_parameter(
            'joy_timeout',
            0.5
        )

        # =====================================================
        # Get parameters
        # =====================================================

        self.deadzone = self.get_parameter(
            'deadzone'
        ).value

        self.max_rov_yaw_rate = self.get_parameter(
            'max_rov_yaw_rate'
        ).value

        self.max_auv_speed = self.get_parameter(
            'max_auv_speed'
        ).value

        self.heading_adjust_rate = self.get_parameter(
            'heading_adjust_rate'
        ).value

        self.depth_adjust_rate = self.get_parameter(
            'depth_adjust_rate'
        ).value

        self.max_depth = self.get_parameter(
            'max_depth'
        ).value

        self.joy_timeout = self.get_parameter(
            'joy_timeout'
        ).value

        # =====================================================
        # Current control mode
        # =====================================================

        # Start safely in manual mode
        self.control_mode = 'ROV'

        # Previous A button state for edge detection
        self.last_mode_button = 0

        # =====================================================
        # Latest joystick state
        # =====================================================

        self.forward_axis = 0.0
        self.yaw_axis = 0.0
        self.pitch_axis = 0.0

        # =====================================================
        # Vehicle state
        # =====================================================

        self.current_depth = 0.0
        self.current_heading = 0.0

        self.odom_received = False

        # =====================================================
        # AUV targets
        # =====================================================

        self.target_depth = 0.0
        self.target_heading = 0.0
        self.target_speed = 0.0

        # =====================================================
        # Timing
        # =====================================================

        self.last_joy_time = None
        self.last_control_time = self.get_clock().now()

        self.failsafe_active = False

        # =====================================================
        # Publishers
        # =====================================================

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/auv/cmd_vel',
            10
        )

        self.mode_pub = self.create_publisher(
            String,
            '/auv/control_mode',
            10
        )

        self.depth_pub = self.create_publisher(
            Float64,
            '/auv/target_depth',
            10
        )

        self.heading_pub = self.create_publisher(
            Float64,
            '/auv/target_heading',
            10
        )

        self.speed_pub = self.create_publisher(
            Float64,
            '/auv/target_speed',
            10
        )

        # =====================================================
        # Subscribers
        # =====================================================

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/auv/odom',
            self.odom_callback,
            10
        )

        # =====================================================
        # Timers
        # =====================================================

        # 20 Hz control loop
        self.control_timer = self.create_timer(
            0.05,
            self.control_loop
        )

        # Re-publish mode occasionally so both nodes remain
        # synchronized even if one starts slightly later.
        self.mode_timer = self.create_timer(
            1.0,
            self.publish_mode
        )

        self.get_logger().info(
            'Gamepad controller started in ROV mode'
        )

    # =========================================================
    # Utility
    # =========================================================

    def clamp(self, value, minimum, maximum):

        return max(
            minimum,
            min(value, maximum)
        )

    def apply_deadzone(self, value):

        if abs(value) < self.deadzone:
            return 0.0

        return value

    def wrap_angle(self, angle):

        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    # =========================================================
    # Mode publishing
    # =========================================================

    def publish_mode(self):

        msg = String()
        msg.data = self.control_mode

        self.mode_pub.publish(msg)

    # =========================================================
    # Odometry
    # =========================================================

    def odom_callback(self, msg):

        # Depth:
        # Gazebo Z decreases as vehicle goes deeper
        self.current_depth = (
            -msg.pose.pose.position.z
        )

        # Quaternion -> yaw / heading
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

        self.odom_received = True

    # =========================================================
    # Joystick callback
    # =========================================================

    def joy_callback(self, msg):

        self.last_joy_time = self.get_clock().now()

        # Safety check
        if len(msg.axes) <= self.AXIS_PITCH:
            self.get_logger().warn(
                'Joystick does not contain expected axes'
            )
            return

        if len(msg.buttons) <= self.BUTTON_MODE:
            self.get_logger().warn(
                'Joystick does not contain expected buttons'
            )
            return

        # -----------------------------------------------------
        # Read sticks
        # -----------------------------------------------------

        self.yaw_axis = self.apply_deadzone(
            msg.axes[self.AXIS_YAW]
        )

        self.forward_axis = self.apply_deadzone(
            msg.axes[self.AXIS_FORWARD]
        )

        self.pitch_axis = self.apply_deadzone(
            msg.axes[self.AXIS_PITCH]
        )

        # -----------------------------------------------------
        # A button mode toggle
        # -----------------------------------------------------

        mode_button = msg.buttons[
            self.BUTTON_MODE
        ]

        # Rising edge only:
        # 0 -> 1
        if (
            mode_button == 1
            and self.last_mode_button == 0
        ):

            self.toggle_mode()

        self.last_mode_button = mode_button

        # -----------------------------------------------------
        # ROV commands can be sent directly
        # -----------------------------------------------------

        if self.control_mode == 'ROV':

            self.publish_rov_command()

    # =========================================================
    # Mode switch
    # =========================================================

    def toggle_mode(self):

        if self.control_mode == 'ROV':

            if not self.odom_received:

                self.get_logger().warn(
                    'Cannot enter AUV mode yet: no odometry received'
                )

                return

            self.control_mode = 'AUV'

            # Capture current state so switching modes
            # does not suddenly command a maneuver.
            self.target_depth = self.current_depth
            self.target_heading = self.current_heading
            self.target_speed = 0.0

            self.publish_mode()

            self.publish_auv_targets()

            self.get_logger().info(
                '=============================='
            )

            self.get_logger().info(
                'CONTROL MODE: AUV'
            )

            self.get_logger().info(
                f'Holding depth: '
                f'{self.target_depth:.2f} m'
            )

            self.get_logger().info(
                f'Holding heading: '
                f'{math.degrees(self.target_heading):.1f} deg'
            )

            self.get_logger().info(
                '=============================='
            )

        else:

            self.control_mode = 'ROV'

            # Stop autonomous speed command
            self.target_speed = 0.0

            speed_msg = Float64()
            speed_msg.data = 0.0

            self.speed_pub.publish(
                speed_msg
            )

            self.publish_mode()

            # Send neutral command immediately
            neutral = Twist()

            self.cmd_vel_pub.publish(
                neutral
            )

            self.get_logger().info(
                '=============================='
            )

            self.get_logger().info(
                'CONTROL MODE: ROV'
            )

            self.get_logger().info(
                '=============================='
            )

    # =========================================================
    # ROV mode
    # =========================================================

    def publish_rov_command(self):

        cmd = Twist()

        # Left stick Y:
        # +1 forward
        # -1 reverse
        cmd.linear.x = self.forward_axis

        # Right stick Y:
        #
        # Your thruster controller already converts
        # angular.y into desired pitch.
        #
        # We invert here so pushing the stick forward
        # commands the nose downward / dive direction.
        cmd.angular.y = -self.pitch_axis

        # Left stick X:
        #
        # Typical Linux gamepad:
        # stick left  -> +1
        # stick right -> -1
        #
        # ROS yaw:
        # + = left / counter-clockwise
        # - = right / clockwise
        #
        # Therefore the raw axis direction already fits.
        cmd.angular.z = (
            self.yaw_axis *
            self.max_rov_yaw_rate
        )

        self.cmd_vel_pub.publish(
            cmd
        )

    # =========================================================
    # AUV mode targets
    # =========================================================

    def publish_auv_targets(self):

        speed_msg = Float64()
        depth_msg = Float64()
        heading_msg = Float64()

        speed_msg.data = self.target_speed
        depth_msg.data = self.target_depth
        heading_msg.data = self.target_heading

        self.speed_pub.publish(
            speed_msg
        )

        self.depth_pub.publish(
            depth_msg
        )

        self.heading_pub.publish(
            heading_msg
        )

    # =========================================================
    # Main 20 Hz control loop
    # =========================================================

    def control_loop(self):

        now = self.get_clock().now()

        dt = (
            now -
            self.last_control_time
        ).nanoseconds * 1e-9

        self.last_control_time = now

        dt = self.clamp(
            dt,
            0.0,
            0.1
        )

        # -----------------------------------------------------
        # Joystick timeout / failsafe
        # -----------------------------------------------------

        if self.last_joy_time is None:

            return

        joy_age = (
            now -
            self.last_joy_time
        ).nanoseconds * 1e-9

        if joy_age > self.joy_timeout:

            if not self.failsafe_active:

                self.get_logger().warn(
                    'Gamepad timeout - stopping forward command'
                )

                self.failsafe_active = True

            if self.control_mode == 'ROV':

                # Zero throttle, pitch and yaw commands
                neutral = Twist()

                self.cmd_vel_pub.publish(
                    neutral
                )

            else:

                # In AUV mode:
                #
                # stop forward motion,
                # but KEEP depth and heading hold.
                self.target_speed = 0.0

                self.publish_auv_targets()

            return

        # Joystick is alive again
        if self.failsafe_active:

            self.get_logger().info(
                'Gamepad connection restored'
            )

            self.failsafe_active = False

        # -----------------------------------------------------
        # Only autonomous target processing below this point
        # -----------------------------------------------------

        if self.control_mode != 'AUV':

            return

        # =====================================================
        # AUV SPEED
        # =====================================================
        #
        # Left Stick Y directly commands desired speed.
        #
        # +1 -> +max speed
        #  0 -> 0 m/s
        # -1 -> reverse max speed

        self.target_speed = (
            self.forward_axis *
            self.max_auv_speed
        )

        # =====================================================
        # AUV HEADING
        # =====================================================
        #
        # Left Stick X moves the heading target.
        #
        # Hold stick -> rotate target heading
        # Release    -> hold heading

        heading_change = (
            self.yaw_axis *
            self.heading_adjust_rate *
            dt
        )

        self.target_heading = self.wrap_angle(
            self.target_heading +
            heading_change
        )

        # =====================================================
        # AUV DEPTH
        # =====================================================
        #
        # Right Stick forward:
        # increase depth -> dive
        #
        # Right Stick backward:
        # decrease depth -> climb

        depth_change = (
            self.pitch_axis *
            self.depth_adjust_rate *
            dt
        )

        self.target_depth += depth_change

        self.target_depth = self.clamp(
            self.target_depth,
            0.0,
            self.max_depth
        )

        # =====================================================
        # Publish autonomous targets
        # =====================================================

        self.publish_auv_targets()


def main(args=None):

    rclpy.init(args=args)

    node = GamepadController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    # Send neutral ROV command before shutdown
    neutral = Twist()

    node.cmd_vel_pub.publish(
        neutral
    )

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()