import os
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, IncludeLaunchDescription,
                             TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg = get_package_share_directory('bcr_bot_with_arm')
    bcr_bot_pkg = get_package_share_directory('bcr_bot')
    gz_sim_share = get_package_share_directory('ros_gz_sim')

    controllers_path = os.path.join(pkg, 'config', 'ros2_controllers.yaml')
    urdf_path = os.path.join(pkg, 'urdf', 'bcr_bot_with_arm.urdf.xacro')
    world_file = os.path.join(bcr_bot_pkg, 'worlds', 'small_warehouse.sdf')

    robot_desc = xacro.process_file(urdf_path, mappings={
        'two_d_lidar_enabled': 'true',
        'camera_enabled': 'true',
        'odometry_source': 'world',
        'ros2_controllers_path': controllers_path,
    }).toxml()

    set_resource_path_worlds = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', value=os.path.join(bcr_bot_pkg, 'worlds'))
    set_resource_path_models = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', value=os.path.join(bcr_bot_pkg, 'models'))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': PythonExpression(["'", world_file, " -r'"])
        }.items()
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen'
    )

    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', '/robot_description', '-name', 'bcr_bot',
                       '-z', '0.28', '-x', '0.0', '-y', '0.0'],
            output='screen'
        )]
    )

    gz_bridge = TimerAction(
        period=8.0,
        actions=[Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/world/default/model/bcr_bot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/arm_joint1_cmd@std_msgs/msg/Float64]gz.msgs.Double',
                '/arm_joint2_cmd@std_msgs/msg/Float64]gz.msgs.Double',
                '/arm_joint3_cmd@std_msgs/msg/Float64]gz.msgs.Double',
                '/left_finger_cmd@std_msgs/msg/Float64]gz.msgs.Double',
                '/right_finger_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            ],
            remappings=[
                ('/world/default/model/bcr_bot/joint_state', 'bcr_bot/joint_states'),
                ('/odom', 'bcr_bot/odom'),
                ('/scan', 'bcr_bot/scan'),
                ('/imu', 'bcr_bot/imu'),
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '-16.344', '--y', '-14.035', '--z', '0.0',
                   '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': True}]
    )

    scan_relay = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/bcr_bot/scan', '/scan'],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        set_resource_path_worlds,
        set_resource_path_models,
        gz_sim,
        rsp,
        static_tf,
        scan_relay,
        spawn_robot,
        gz_bridge,
    ])
