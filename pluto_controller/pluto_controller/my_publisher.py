#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pluto_interfaces.msg import TargetLight


class MyPublisherNode(Node):

    def __init__(self):
        super().__init__("my_publisher")

        #Message class, Name of topic, Queue message size
        self.publisher_ = self.create_publisher(TargetLight, "topic",10)
        # Change every 7 seconds
        self.timer_ = self.create_timer(5, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        msg = TargetLight()
        msg.data= "This is my published message " + str(self.i)

        
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing a message")
        self.i += 1



def main(args=None):
    rclpy.init(args=args)
    node = MyPublisherNode()

    rclpy.spin(node)

    #destroy node and shutdown ros2 communication
    rclpy.shutdown()