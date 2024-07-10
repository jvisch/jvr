from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Also in gazebo.launch.py declared (TODO: how to pass around)
world = 'jvr_empty_world'
robot_name = 'jvr'

# share directory
pkg_jvr_simulation = get_package_share_directory('jvr_sim')


def launch_simulation_callback(context, *args, **kwargs):

    # RViz2
    # rviz configuration file
    rviz2_config_file = PathJoinSubstitution(
        [pkg_jvr_simulation, 'rviz', 'jvr.rviz'])
    # RViz node
    node_rviz2 = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_config_file.perform(context)]
    )

    # A bridge to forward tf and joint states to ros2
    node_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            f'/world/{world}/model/{
                robot_name}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # f'/model/{robot_name}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        remappings=[
            # (f'/model/{robot_name}/pose', '/tf'),
            (f'/world/{world}/model/{robot_name}/joint_state', '/joint_states')
        ]
    )

    return [
        node_rviz2,
        node_bridge
    ]


def generate_launch_description():
    import os
    p = PythonLaunchDescriptionSource(
        os.path.join(
            pkg_jvr_simulation,
            "launch/gazebo.launch.py")
    )
    x = IncludeLaunchDescription(p)

    return LaunchDescription([
        OpaqueFunction(function=launch_simulation_callback),
        x
    ])
