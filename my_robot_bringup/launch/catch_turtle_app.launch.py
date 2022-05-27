from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
    )
    ld.add_action(turtlesim_node)

    turtle_spawner_node = Node(
        package="turtle_catch_them_all",
        executable="turtle_spawner",
        parameters=[
            {"spawn_period": 0.5},
            {"max_number_of_alive_turtles": 10}
        ]
    )
    ld.add_action(turtle_spawner_node)

    turtle_controller_node = Node(
        package="turtle_catch_them_all",
        executable="turtle_controller",
        parameters=[
            {"catch_nearest_first": False}
        ]
    )
    ld.add_action(turtle_controller_node)

    return ld
