#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Lidar args (forwarded to rplidar_ros launch)
    lidar_launch_file = LaunchConfiguration('lidar_launch_file')
    channel_type = LaunchConfiguration('channel_type')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    inverted = LaunchConfiguration('inverted')
    angle_compensate = LaunchConfiguration('angle_compensate')

    # Chassis args
    chassis_params_file = LaunchConfiguration('chassis_params_file')

    # slam_toolbox args
    slam_params_file = LaunchConfiguration('slam_params_file')
    slam_executable = LaunchConfiguration('slam_executable')
    enable_slam = LaunchConfiguration('enable_slam')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # base_link -> laser static TF args
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_roll = LaunchConfiguration('laser_roll')
    laser_pitch = LaunchConfiguration('laser_pitch')
    laser_yaw = LaunchConfiguration('laser_yaw')
    base_frame = LaunchConfiguration('base_frame')
    laser_frame = LaunchConfiguration('laser_frame')

    rplidar_share = FindPackageShare('rplidar_ros')
    little_chassis_share = FindPackageShare('little_chassis')
    bringup_share = FindPackageShare('bringup')

    lidar_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rplidar_share, 'launch', lidar_launch_file])
        ),
        launch_arguments={
            'channel_type': channel_type,
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': lidar_frame_id,
            'inverted': inverted,
            'angle_compensate': angle_compensate,
        }.items(),
    )

    chassis_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([little_chassis_share, 'launch', 'little_chassis.launch.py'])
        ),
        launch_arguments={
            'params_file': chassis_params_file,
        }.items(),
    )

    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf_pub',
        arguments=[
            laser_x,
            laser_y,
            laser_z,
            laser_roll,
            laser_pitch,
            laser_yaw,
            base_frame,
            laser_frame,
        ],
        output='screen',
    )

    odom_to_tf = Node(
        package='bringup',
        executable='odom_to_tf',
        name='odom_to_tf',
        parameters=[{
            'odom_topic': 'odom',
            'publish_tf': True,
        }],
        output='screen',
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable=slam_executable,
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(enable_slam),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_launch_file',
            default_value='rplidar_a1_launch.py',
            description='Which rplidar_ros launch file to include (e.g. rplidar_s2_launch.py).',
        ),
        DeclareLaunchArgument('channel_type', default_value='serial'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('lidar_frame_id', default_value='laser'),
        DeclareLaunchArgument('inverted', default_value='false'),
        DeclareLaunchArgument('angle_compensate', default_value='true'),

        DeclareLaunchArgument(
            'chassis_params_file',
            default_value=PathJoinSubstitution([little_chassis_share, 'params', 'chassis.yaml']),
            description='Full path to little_chassis params yaml',
        ),

        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([bringup_share, 'config', 'slam_toolbox.yaml']),
            description='Full path to slam_toolbox params yaml',
        ),
        DeclareLaunchArgument(
            'slam_executable',
            default_value='async_slam_toolbox_node',
            description='slam_toolbox executable to run (e.g. async_slam_toolbox_node or sync_slam_toolbox_node).',
        ),
        DeclareLaunchArgument(
            'enable_slam',
            default_value='true',
            description='Whether to start slam_toolbox.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('laser_frame', default_value='laser'),
        DeclareLaunchArgument('laser_x', default_value='0.0'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.0'),
        DeclareLaunchArgument('laser_roll', default_value='0.0'),
        DeclareLaunchArgument('laser_pitch', default_value='0.0'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0'),

        lidar_include,
        chassis_include,
        odom_to_tf,
        base_to_laser_tf,
        slam_toolbox_node,
    ])
