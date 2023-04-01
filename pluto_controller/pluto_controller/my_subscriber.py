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


    def listener_callback(self,msg):
        payload = msg.data
        move = Twist()
        ticks = WheelTicks()
        vels = WheelVels()
        status = WheelStatus()

        move = msg.twst
        ticks = msg.tick
        vels = msg.vel
        status = msg.status

        self.get_logger().info("Message recieved...moving " + str(move))
        self.cmd_move_pub.publish(move)
        
        self.get_logger().info("Pluto ticks left/right: " + str(ticks.ticks_left) +" : "+ str(ticks.ticks_right))
        self.get_logger().info("Pluto vels left/right: " + str(vels.velocity_left) +" : "+ str(vels.velocity_right))
        self.get_logger().info("Pluto status: " + str(status.wheels_enabled))
def main(args=None):
    rclpy.init(args=args)
    node = MySubscriberNode()

    #Spin means the node will keep running until killed
    rclpy.spin(node)
    
    #destroy node and shutdown ros2 communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()