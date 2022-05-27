#!/usr/bin/env python3
from tarfile import TarError
from turtle import distance
import rclpy
import math
from rclpy.node import Node
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

def manhattan_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller")
        self.declare_parameter("catch_nearest_first", True)
        self.catch_nearest_first = self.get_parameter("catch_nearest_first").value

        self.master_turtle_pose = None
        self.target_turtle = None
        
        self.active_turtle_subcriber_ = self.create_subscription(TurtleArray, "alive_turtles", self.received_active_turtles, 10)
        self.turtle1_pose_subcriber_ = self.create_subscription(Pose, "turtle1/pose", self.received_master_turtle_pose, 10)
        self.publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
    
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)
     
        self.get_logger().info("turtle controller started")

    def received_master_turtle_pose(self, msg):
        self.master_turtle_pose = (msg.x, msg.y, msg.theta)
        #self.get_logger().info(f"current master turtle is at ({msg.x}, {msg.y})")

    def received_active_turtles(self, msg):
        if self.target_turtle is not None:
            self.get_logger().info("stop processing when we are in the process of targeting one")
            return 
        
        if self.master_turtle_pose is None:
            self.get_logger().info("stop processing no master turtle")
            return

        if not self.catch_nearest_first:
            self.target_turtle = msg.list[0]
            return 

        current_x, current_y, _ = self.master_turtle_pose
        best_distance = None
        for turtle in msg.list:
            distance = manhattan_distance(current_x, current_y, turtle.x, turtle.y)
            if best_distance is None:
                best_distance = distance
                self.target_turtle = turtle
            elif distance < best_distance:
                best_distance = distance
                self.target_turtle = turtle
    
    def control_loop(self):
        if self.master_turtle_pose is None or self.target_turtle is None:
            return 

        self.get_logger().info(f"going to catch a turtle at {self.target_turtle}")
        
        velocity_msg = Twist()

        current_x, current_y, current_theta = self.master_turtle_pose
        target_x, target_y = self.target_turtle.x, self.target_turtle.y
        distance = manhattan_distance(current_x, current_y, target_x, target_y)
        if distance < 0.5:
            self.kill_turtle(self.target_turtle)
            self.target_turtle = None
            velocity_msg.linear.x = 0.0
            velocity_msg.angular.z = 0.0
        else:
            K_linear, K_angular = 2.0, 4.0
            linear_speed = distance * K_linear
            disired_angle_goal = math.atan2(target_y - current_y, target_x - current_x)
            angular_speed = self.clean_angle(disired_angle_goal - current_theta) * K_angular
            velocity_msg.linear.x = linear_speed
            velocity_msg.angular.z = angular_speed

        self.publisher_.publish(velocity_msg)

    def clean_angle(self, raw_angle):
        ret_angle = raw_angle
        while ret_angle < -math.pi:
            ret_angle += math.pi * 2
        while ret_angle > math.pi:
            ret_angle -= math.pi * 2
        return ret_angle

    def kill_turtle(self, target_turtle):
        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for server CatchTurtle")
        
        request = CatchTurtle.Request()
        request.name = target_turtle.name
        future = client.call_async(request)
        future.add_done_callback(self.kill_callback)

    def kill_callback(self, future):
        self.get_logger().info(f"kill callback")
   

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()