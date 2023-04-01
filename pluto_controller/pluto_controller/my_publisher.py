#!/usr/bin/env python3
import rclpy
import keyboard
from rclpy.node import Node
from pluto_interfaces.msg import TargetLight
from pluto_interfaces.msg import TargetMove
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
        header = self.get_clock().now().to_msg()
        msg.data = self.action

        msg.vel = WheelVels()
        msg.tick = WheelTicks()
        msg.status = WheelStatus()

        msg.vel.header.stamp = header
        msg.vel.velocity_left = 0.5
        msg.vel.velocity_right = 0.5
        
        msg.tick.header.stamp = header
        msg.tick.ticks_left = 1
        msg.tick.ticks_right = 1

        msg.status.header.stamp = header
        msg.status.current_ma_left = 0
        msg.status.current_ma_right =0
        msg.status.pwm_left =0 
        msg.status.pwm_right =0
        msg.status.wheels_enabled = True
        self.movement_publisher_.publish(msg)
        self.get_logger().info("sending a command now " + str(msg))

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