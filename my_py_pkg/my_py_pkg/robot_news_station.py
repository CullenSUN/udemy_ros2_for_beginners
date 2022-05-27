#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String 
 
class RobotNewsStationNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("robot_news_station")
        self.declare_parameter("robot_name", "C3P0")
        self.robot_name_ = self.get_parameter("robot_name").value
        self.publisher_ = self.create_publisher(String, "robot_news", 10)
        self.create_timer(0.5, self.publish_news)
        self.get_logger().info("robot_news_station started")

    def publish_news(self):
        msg = String()
        msg.data = "Hi, this is " + str(self.robot_name_) + " from the robot news station"
        self.publisher_.publish(msg)

    
def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()