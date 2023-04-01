#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import WheelStatus
from pluto_interfaces.msg import TargetLight

class MySubscriberNode(Node):


    def __init__(self):
        super().__init__("my_subscriber")
        self.my_subscriber_ = self.create_subscription(TargetLight,"topic",self.listener_callback,10)
        self.wheel_status_subscriber_  = self.create_subscription(WheelStatus,"/wheel_status",self.status_callback, 10)

    def status_callback(self,msg):
        self.get_logger().info("Status of wheels: " + str(msg))

    def listener_callback(self,msg):
        payload = msg.data
        self.get_logger().info("I heard: " + payload)

def main(args=None):
    rclpy.init(args=args)
    node = MySubscriberNode()

    #Spin means the node will keep running until killed
    rclpy.spin(node)
    
    #destroy node and shutdown ros2 communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()