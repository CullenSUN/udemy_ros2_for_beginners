from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    for index in range(5):
        news_publisher = Node(
            package="my_cpp_pkg",
            executable="robot_news_station",
            name= f"robot_news_station_{index}",
            parameters=[
                {"robot_name": f"robot_{index}"}
            ]
        )
        ld.add_action(news_publisher)

    smartphone = Node(
        package="my_cpp_pkg",
        executable="smartphone",
    )
    ld.add_action(smartphone)

    return ld
