from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch.actions import AppendEnvironmentVariable

import os


def generate_launch_description():

    package_dir = get_package_share_directory('auv_sim')

    urdf_file = os.path.join(
        package_dir,
        'urdf',
        'auv.urdf'
    )

    controllers_file = os.path.join(
        package_dir,
        'config',
        'controllers.yaml'
    )

    auv_resource_path = os.path.join(
        get_package_prefix('auv_sim'),
        'share'
    )

    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=auv_resource_path
    )

    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    return LaunchDescription([

         # -------------------------------------------------
        # Gazebo resource path
        # -------------------------------------------------
        set_gz_resource_path,

        # -------------------------------------------------
        # Gazebo Jetty
        # -------------------------------------------------

        # Gz server
        ExecuteProcess(
            cmd=[
                'gz',
                'sim',
                '-r',
                '-s',
                os.path.join(
                    package_dir,
                    'worlds',
                    'auv_world.sdf'
                )
            ],
            output='screen'
        ),

        # Gz GUI
        ExecuteProcess(
            cmd=[
                'env',
                '-u',
                'GTK_PATH',
                'gz',
                'sim',
                '-g'
            ],
            output='screen'
        ),

        # -------------------------------------------------
        # ROS 2 robot_state_publisher
        # -------------------------------------------------

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_description
                }
            ]
        ),

        # -------------------------------------------------
        # Gamepad joystick driver
        # -------------------------------------------------

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[
                {
                    'autorepeat_rate': 20.0
                }
            ]
        ),

        # -------------------------------------------------
        # Gazebo <-> ROS 2 bridge
        # -------------------------------------------------

        ExecuteProcess(
            cmd=[
                'ros2',
                'run',
                'ros_gz_bridge',
                'parameter_bridge',

                # Clock
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

                 # Gazebo -> ROS IMU
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',

                # Gazebo -> ROS Odometry
                '/auv/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',

                # Left propeller
                '/model/auv/joint/left_propeller_joint/cmd_thrust'
                '@std_msgs/msg/Float64]gz.msgs.Double',

                # Right propeller
                '/model/auv/joint/right_propeller_joint/cmd_thrust'
                '@std_msgs/msg/Float64]gz.msgs.Double',
            ],
            output='screen'
        ),

        # -------------------------------------------------
        # Spawn AUV
        # -------------------------------------------------

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-file',
                        urdf_file,
                        '-name',
                        'my_auv',
                        '-z',
                        '-1.0',
                    ],
                    output='screen'
                )
            ]
        ),

        # -------------------------------------------------
        # Joint State Broadcaster
        # -------------------------------------------------

        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'joint_state_broadcaster',
                        '--controller-manager',
                        '/controller_manager',
                        '--param-file',
                        controllers_file
                    ],
                    output='screen'
                )
            ]
        ),

        # -------------------------------------------------
        # Left elevator fin controller
        # -------------------------------------------------

        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'left_fin_controller',
                        '--controller-manager',
                        '/controller_manager',
                        '--param-file',
                        controllers_file
                    ],
                    output='screen'
                )
            ]
        ),

        # -------------------------------------------------
        # Right elevator fin controller
        # -------------------------------------------------

        TimerAction(
            period=7.5,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'right_fin_controller',
                        '--controller-manager',
                        '/controller_manager',
                        '--param-file',
                        controllers_file
                    ],
                    output='screen'
                )
            ]
        ),

        # -------------------------------------------------
        # AUV thrust + pitch/yaw controller
        # -------------------------------------------------

        TimerAction(
            period=9.0,
            actions=[
                Node(
                    package='auv_sim',
                    executable='thruster_controller',
                    name='thruster_controller',
                    output='screen'
                )
            ]
        ),

        # -------------------------------------------------
        # Gamepad controller
        # -------------------------------------------------

        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='auv_sim',
                    executable='gamepad_controller',
                    name='gamepad_controller',
                    output='screen'
                )
            ]
        ),

    ])