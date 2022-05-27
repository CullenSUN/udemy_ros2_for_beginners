#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from functools import partial
import random
import math

class TurtleSpawnerNode(Node): 

    def __init__(self):
        super().__init__("turtle_spawner") 
        self.declare_parameter("spawn_period", 0.5)
        self.spawn_period = self.get_parameter("spawn_period").value
        self.declare_parameter("max_number_of_alive_turtles", 5)
        self.max_num_turtles = self.get_parameter("max_number_of_alive_turtles").value

        self.turtles_ = []
        self.server_ = self.create_service(CatchTurtle, "catch_turtle", self.catch_turtle)
        self.publisher_ = self.create_publisher(TurtleArray, "alive_turtles", 10)
        self.spawn_timer_ = self.create_timer(self.spawn_period, self.call_spawn_service)
        self.get_logger().info("turtle_spawner has been started")

    def publish_turtles_status(self):
        msg = TurtleArray()
        msg.list = self.turtles_
        self.publisher_.publish(msg)

    def call_spawn_service(self):
        if len(self.turtles_) >= self.max_num_turtles:
            return 
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for server Spawn")
        
        request = Spawn.Request()
        (x, y, theta) = (random.uniform(0.0, 11.0), 
                         random.uniform(0.0, 11.0), 
                         random.uniform(0.0, 2*math.pi))
        request.x = x
        request.y = y
        request.theta = theta
        future = client.call_async(request)
        future.add_done_callback(partial(self.spawn_callback, x=x, y=y))

    def spawn_callback(self, future, x, y):
        try:
            response = future.result()
            new_turtle = Turtle()
            new_turtle.x = x 
            new_turtle.y = y
            new_turtle.name = response.name
            self.turtles_.append(new_turtle)
            self.publish_turtles_status()
            self.get_logger().info(f"created new turtle {response.name} at x: {x}, y: {y}")

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
  
    def catch_turtle(self, request, response):
        self.call_kill_service(request.name)
        return response

    def call_kill_service(self, name):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for server Kill")
        
        request = Kill.Request()
        request.name = name
        future = client.call_async(request)
        future.add_done_callback(partial(self.kill_callback, name=name))

    def kill_callback(self, future, name):
        try:
            future.result()
            match = next(x for x in self.turtles_ if x.name == name)
            self.turtles_.remove(match)
            self.publish_turtles_status()
            self.get_logger().info(f"kill turtle {name}")

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

 
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()