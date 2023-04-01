#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pluto_interfaces.msg import TargetLight
from pluto_interfaces.msg import TargetMove
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import WheelVels
from irobot_create_msgs.msg import WheelTicks
from irobot_create_msgs.msg import WheelStatus


class MyPublisherNode(Node):

    def __init__(self):
        super().__init__("my_publisher")

        #Message class, Name of topic, Queue message size
        self.publisher_ = self.create_publisher(TargetLight, "topic",10)

        self.movement_publisher_ = self.create_publisher(TargetMove,"/target_move",10)
        # Change every 7 seconds
        self.timer_ = self.create_timer(10, self.pub_move)
        self.action = "forward"

    
 
        
    def pub_move(self):
        msg = TargetMove()
        msg.twst.linear.x = 1.0
        msg.twst.angular.z = 2.0
        self.movement_publisher_.publish(msg)
        self.get_logger().info("Sending a command now " + str(msg))


def main(args=None):
    rclpy.init(args=args)
    node = MyPublisherNode()

    rclpy.spin(node)

    #destroy node and shutdown ros2 communication
    rclpy.shutdown()