from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sim_node",
            executable="sim_node",
            parameters=[
                {"human_gui": False},
                {"cv_exposure": 0.8},
                {"cpu_sim": True}
            ]
        ),

        Node(
            package="sim_node",
            executable="keyboard_controls"
        ),

        Node(
            package="huskybot_cv",
            executable="huskybot_cv",
            parameters=[
                {"use_sim_time": True}
            ]
        ),

        Node(
            package="your_solution",
            executable="tf_broadcaster",
            parameters=[
                {"use_sim_time": True}
            ]
        ),

        Node(
            package="your_solution",
            executable="calc_error",
            parameters=[
                {"use_sim_time": True}
            ]
        )

    ])