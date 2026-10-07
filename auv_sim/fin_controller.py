#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


class FinController(Node):

    def __init__(self):
        super().__init__('fin_controller')

        self.declare_parameter('max_fin_angle', 0.5)
        self.max_fin_angle = self.get_parameter('max_fin_angle').value

        # Receive a desired pitch command
        self.pitch_subscriber = self.create_subscription(
            Float64,
            '/auv/pitch_command',
            self.pitch_callback,
            10
        )

        # Send commands to both elevator fins
        self.fin_publisher = self.create_publisher(
            Float64MultiArray,
            '/left_fin_controller/commands',
            10
        )

        self.get_logger().info('Fin controller started.')

    def pitch_callback(self, msg):
        pitch_angle = msg.data

        # Keep the command inside our joint limits
        pitch_angle = max(
            -self.max_fin_angle,
            min(self.max_fin_angle, pitch_angle)
        )

        command = Float64MultiArray()
        command.data = [pitch_angle, pitch_angle]

        self.fin_publisher.publish(command)

        self.get_logger().info(
            f'Pitch command: {pitch_angle:.3f} rad'
        )


def main(args=None):
    rclpy.init(args=args)

    node = FinController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()