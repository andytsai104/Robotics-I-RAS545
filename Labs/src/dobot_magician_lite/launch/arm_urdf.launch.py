import os
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path



def generate_launch_description():
    # Define launch arguments
    arg_x = DeclareLaunchArgument('arg_x', default_value='0.00', description='X position')
    arg_y = DeclareLaunchArgument('arg_y', default_value='0.00', description='Y position')
    arg_z = DeclareLaunchArgument('arg_z', default_value='0.00', description='Z position')
    arg_R = DeclareLaunchArgument('arg_R', default_value='0.00', description='Roll')
    arg_P = DeclareLaunchArgument('arg_P', default_value='0.00', description='Pitch')
    arg_Y = DeclareLaunchArgument('arg_Y', default_value='0.00', description='Yaw')

    # Get package and file paths
    package_name = 'dobot_magician_lite'
    urdf_file = os.path.join(
        get_package_share_path(package_name),
        'urdf',
        'dobot_magician_lite.urdf'
    )
    controller_yaml = os.path.join(
        get_package_share_path(package_name),
        'config',
        'joint_trajectory_controller.yaml'
    )
    
    # Load Gazebo empty world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_path('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ])
    )

    # Static transform publisher
    tf_static = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    # Spawn URDF model in Gazebo
    spawn_urdf = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'dobot_magician_lite',
            '-x', LaunchConfiguration('arg_x'),
            '-y', LaunchConfiguration('arg_y'),
            '-z', LaunchConfiguration('arg_z'),
            '-Y', LaunchConfiguration('arg_Y'),
            '-file', urdf_file
        ],
        output='screen'
    )


    # Load joint trajectory controller
    load_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_controller', 'robot_arm_controller', 'hand_ee_controller'],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file).read()}],
        output='screen'
    )

    return LaunchDescription([
        arg_x, arg_y, arg_z, arg_R, arg_P, arg_Y,
        gazebo_launch,
        tf_static,
        spawn_urdf,
        load_controllers,
        robot_state_publisher
    ])
