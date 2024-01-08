import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro

use_sim_time = True
gazebo_version = '8'  # harmonic == 8
world = 'empty'

robot_name = 'jvr'
robot_desc_topic = '/robot_description'
robot_xyz = (0, 0, 0.1)
robot_pry = (0, 0, 0)


def launch_simulation_callback(context, *args, **kwargs):

    # share directory
    pkg_jvr_simulation = get_package_share_directory(
        'gazebo-tutorial')  # change this!!
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    ## Robot State Publisher #########################
    #  Read robot description (xacro)
    file_name_urdf = PathJoinSubstitution(
        [pkg_jvr_simulation, 'description', 'jvr.urdf.xacro']).perform(context)
    robot_desc = xacro.process_file(file_name_urdf).toxml()
    # Create node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_desc}]
    )

    ## Launch Gazebo with World ######################
    #  World file
    world_file = PathJoinSubstitution(
        [pkg_jvr_simulation, 'worlds', f'{world}.sdf'])
    # Simulation launchfile
    gz_sim_launch_file = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    # Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_file),
        launch_arguments={
            'gz_args': world_file,
            'gz_version': gazebo_version
        }.items()
    )

    ## Publish JVR robot in world ####################
    # position and pose
    x, y, z = map(str, robot_xyz)  # convert all to strings
    p, r, y = map(str, robot_pry)
    # Ros GZ sim node
    node_create_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='create_robot',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-world', world,
                   '-topic', robot_desc_topic,
                   '-name', robot_name,
                   '-x', x,
                   '-y', y,
                   '-z', z,
                   '-P', p,
                   '-R', r,
                   '-Y', y
                   ]
    )

    ## RViz2
    # rviz configuration file
    rviz2_config_file = PathJoinSubstitution([pkg_jvr_simulation, 'rviz', 'jvr.rviz'])
    # RViz node
    node_rviz2 = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_config_file.perform(context)]
    )

    # Bridge to forward tf and joint states to ros2
    gz_topic = f'/model/{robot_name}'
    joint_state_gz_topic = f'/world/{world}' + gz_topic + '/joint_state'
    link_pose_gz_topic = gz_topic + '/pose'
    node_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (Gazebo -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # Joint states (Gazebo -> ROS2)
            joint_state_gz_topic + '@sensor_msgs/msg/JointState[gz.msgs.Model',
            
            # Link poses (Gazebo -> ROS2)
            link_pose_gz_topic + '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            link_pose_gz_topic + '_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            
            # Velocity and odometry (Gazebo -> ROS2)
            gz_topic + '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            gz_topic + '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        remappings=[
            (joint_state_gz_topic, 'joint_states'),
            (link_pose_gz_topic, '/tf'),
            (link_pose_gz_topic + '_static', '/tf_static'),
        ],
        parameters=[{'qos_overrides./tf_static.publisher.durability': 'transient_local'}],
        output='screen'
    )

    return [
        gz_sim,
        node_rviz2,
        node_robot_state_publisher,
        node_create_robot,
        node_bridge
    ]


def generate_launch_description():

    # # rviz configuration file
    # # rviz_file_name = "description/jvr.rviz"
    # # rviz_config = os.path.join(directory, rviz_file_name)

    # Robot State Publisher

    # return LaunchDescription([
    #     DeclareLaunchArgument(
    #         'use_sim_time',
    #         default_value='false',
    #         description='Use simulation (Gazebo) clock if true'),
    #     Node(
    #         package='robot_state_publisher',
    #         executable='robot_state_publisher',
    #         name='robot_state_publisher',
    #         output='screen',
    #         parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}]
    #         ),
    #     # Node(
    #     #     package='joint_state_publisher_gui',
    #     #     executable='joint_state_publisher_gui',
    #     #     name='joint_state_publisher_gui',
    #     #     output='screen'),
    #     gz_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    #     launch_arguments={'gz_args': PathJoinSubstitution([
    #         pkg_project_gazebo,
    #         'worlds',
    #         'diff_drive.sdf'
    #     ])}.items(),
    # )
    #     Node(
    #         package='rviz2',
    #         executable='rviz2',
    #         name='rviz2',
    #         output='screen',
    #         arguments=["--display-config", rviz_config]),
    # ])

    return LaunchDescription([
        OpaqueFunction(function=launch_simulation_callback)
    ])
