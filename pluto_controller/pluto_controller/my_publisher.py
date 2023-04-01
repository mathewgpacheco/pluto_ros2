#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pluto_interfaces.msg import TargetMove
from pluto_interfaces.msg import DetectSensors

class MyPublisherNode(Node):

    def __init__(self):
        super().__init__("my_publisher")
        
        self.movement_publisher_ = self.create_publisher(TargetMove,"/target_move",10)
        self.detect_subscriber_ = self.create_subscription(DetectSensors, "/detect_sensors",self.listener_callback,10)
    
    def listener_callback(self,msg):
        if msg.buttons.button_1.is_pressed:
            btn_pressed = "left"
            self.get_logger().info("Button press: " + btn_pressed)
        if msg.buttons.button_2.is_pressed:
            btn_pressed = "right"
            self.get_logger().info("Button press: " + btn_pressed)
        else:
            self.get_logger().info("Button press: None")

    def publish_move(self):
        msg = TargetMove()
        msg.twst.linear.x = 1.0
        msg.twst.angular.z = 2.0
        msg.data = self.action
        self.movement_publisher_.publish(msg)
        self.get_logger().info("Sending a command now " + str(msg))


def main(args=None):
    rclpy.init(args=args)
    node = MyPublisherNode()

    rclpy.spin(node)

    #destroy node and shutdown ros2 communication
    rclpy.shutdown()