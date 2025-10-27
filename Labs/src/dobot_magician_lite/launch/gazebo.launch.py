import os
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Get package paths
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    dobot_pkg = get_package_share_directory('dobot_magician_lite')

    # Launch Gazebo with an empty world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
    )

    # Static transform publisher (base_footprint -> base_link)
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    # Spawn the Dobot Magician Lite URDF model
    urdf_file_path = os.path.join(dobot_pkg, 'urdf', 'dobot_magician_lite.urdf')
    
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', urdf_file_path, '-entity', 'dobot_magician_lite'],
        output='screen'
    )

    # # Publish "calibrated" message (Equivalent to "rostopic pub /calibrated std_msgs/Bool true" in ROS 1)
    # fake_joint_calibration = Node(
    #     package='std_msgs',
    #     executable='pub',
    #     arguments=['/calibrated', 'std_msgs/msg/Bool', '{data: true}'],
    #     output='screen'
    # )

    fake_joint_calibration = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/calibrated', 'std_msgs/msg/Bool', '{data: true}'],
        output='screen'
    )


    return LaunchDescription([
        gazebo_launch,
        static_tf_publisher,
        spawn_robot,
        fake_joint_calibration
    ])
