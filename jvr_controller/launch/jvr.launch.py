import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'jvr.urdf'
    urdf = os.path.join(
        get_package_share_directory('jvr_controller'),
        urdf_file_name)
    print(urdf)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='jvr_controller',
            executable='robot_position_publisher'),
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui'),
        Node(
            package='rviz2',
            executable='rviz2'
        )
    ])