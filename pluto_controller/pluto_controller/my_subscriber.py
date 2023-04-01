#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import InterfaceButtons
from geometry_msgs.msg import Twist
from pluto_interfaces.msg import TargetMove
from pluto_interfaces.msg import DetectSensors

class MySubscriberNode(Node):


    def __init__(self):
        super().__init__("my_subscriber")

        #publish an actual move
        self.cmd_move_pub = self.create_publisher(Twist,"/cmd_vel",10)

        #subscribe to interface buttons; used to change state
        self.button_subscriber_ = self.create_subscription(InterfaceButtons,"/interface_buttons", self.button_callback,10)
        
        #publish sense detections back to command
        self.detect_publisher_ = self.create_publisher(DetectSensors, "/detect_sensors",10)


        #subscribe to movement actions commands from command
        self.movement_subscriber_ = self.create_subscription(TargetMove, "/target_move", self.listener_callback,10)

    def button_callback(self,msg):

        if msg.button_2.is_pressed:
            self.get_logger().info("right button pressed")
            self.detect_publisher_.publish(msg)
        
        if msg.button_1.is_pressed:
            self.get_logger().info("left button pressed")
            self.detect_publisher_.publish(msg)
        


    def listener_callback(self,msg):
        payload = msg.data
        move = Twist()
        move = msg.twst
        self.get_logger().info("Message recieved...moving ")
        self.cmd_move_pub.publish(move)


def main(args=None):
    rclpy.init(args=args)
    node = MySubscriberNode()

    #Spin means the node will keep running until killed
    rclpy.spin(node)
    
    #destroy node and shutdown ros2 communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()