from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # RELbot Simulator Node
        Node(
            package="relbot_simulator",
            executable="relbot_simulator",
            name="relbot_simulator",
            output="screen"
        ),

        # Moving Camera Tracker Node
        Node(
            package="moving_camera_tracker",
            executable="moving_camera_tracker_node",
            name="moving_camera_tracker",
            output="screen"
        ),

        # Controler Node
        Node(
            package="controler_1_2_3",
            executable="controler_1_2_3_node",
            name="controler_1_2_3",
            output="screen"
        )
    ])
