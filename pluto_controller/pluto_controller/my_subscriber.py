#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from irobot_create_msgs.msg import WheelVels
from irobot_create_msgs.msg import WheelTicks
from irobot_create_msgs.msg import WheelStatus
from geometry_msgs.msg import Twist
from pluto_interfaces.msg import TargetMove

class MySubscriberNode(Node):


    def __init__(self):
        super().__init__("my_subscriber")

        #publish an actual move
        self.cmd_move_pub = self.create_publisher(Twist,"/cmd_vel",10)

        #subscribe to movement actions commands
        self.movement_subscriber_ = self.create_subscription(TargetMove, "/target_move", self.listener_callback,10)

        self.movement_velocity_publisher_ = self.create_publisher(WheelVels,"/wheel_vels", 10)
        self.timer_ = self.create_timer(10, self.timer_callback)
        
    def timer_callback(self):
        msg = WheelVels()

        msg.velocity_left = 2.0
        msg.velocity_right = 2.0
        self.movement_velocity_publisher_.publish(msg)
        self.get_logger().info("Pluto velocity left/right " + str(msg.velocity_left) + " : " + str(msg.velocity_right))


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