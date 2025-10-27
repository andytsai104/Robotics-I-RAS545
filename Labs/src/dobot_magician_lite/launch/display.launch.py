import os
import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get package path
    dobot_pkg = get_package_share_directory('dobot_magician_lite')

    # Declare argument for model file
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(dobot_pkg, 'urdf', 'dobot_magician_lite.urdf'),
        description='Path to robot URDF file'
    )

    # Static transform from "odom" to "base_link"
    static_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )


    # Convert URDF using Xacro and pass as a string parameter
    robot_description_param = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', LaunchConfiguration('model')]),
                value_type=str
            )
        }],
        output='screen'
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # Load RViz with a configuration file
    rviz_config_file = os.path.join(dobot_pkg, 'config', 'urdf.rviz')

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        robot_description_param,
        joint_state_publisher_gui,
        rviz_node,
        static_tf
    ])
