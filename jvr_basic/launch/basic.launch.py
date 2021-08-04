from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    talker = Node(
        package='jvr_basic', 
        executable='talker'
    )
    listener = Node(
        package='jvr_basic', 
        executable='listener'
    )

    launcher = LaunchDescription()
    
    launcher.add_entity(talker)
    launcher.add_entity(listener)

    return launcher