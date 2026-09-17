import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('skid_steer_odom')
    ros_ign_gazebo_share = get_package_share_directory('ros_ign_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')
    bridge_config_file = os.path.join(pkg_share, 'config', 'bridge_config.yaml')

    xacro_file = os.path.join(pkg_share, 'urdf', 'rover.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    # 1. Gazebo
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_ign_gazebo_share, 'launch', 'ign_gazebo.launch.py')
        ),
        launch_arguments={'ign_args': '-r empty.sdf'}.items()
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # 3. Spawn Rover
    spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-name', 'skid_steer_rover',
            '-topic', 'robot_description',
            '-z', '0.15'
        ],
        output='screen'
    )

    # 4. Parameter Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config_file}'],
        output='screen'
    )

    # 5. Encoder Emulator with 2% Right-Wheel Asymmetry
    encoder_emulator = Node(
        package='skid_steer_odom',
        executable='encoder_emulator.py',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'right_wheel_scale': 1.02  # Part 6: 2% larger effective wheel scale
        }]
    )

    # 6. Calibrated Wheel Odometry
    wheel_odometry = Node(
        package='skid_steer_odom',
        executable='wheel_odometry.py',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        ign_gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        encoder_emulator,
        wheel_odometry
    ])