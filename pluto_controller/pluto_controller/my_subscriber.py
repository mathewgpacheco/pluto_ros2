#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from irobot_create_msgs.msg import WheelVels
from irobot_create_msgs.msg import WheelTicks
from irobot_create_msgs.msg import WheelStatus

from pluto_interfaces.msg import TargetLight
from pluto_interfaces.msg import TargetMove

class MySubscriberNode(Node):


    def __init__(self):
        super().__init__("my_subscriber")
        #self.my_subscriber_ = self.create_subscription(TargetLight,"topic",self.listener_callback,10)


        #subscribe to movement commands
        self.movement_subscriber_ = self.create_subscription(TargetMove, "/target_move", self.listener_callback,10)
        self.get_logger().info("Move subscriber initialized")

        #publish movement actions to robot
        self.cmd_vel_pub = self.create_publisher(WheelVels,"/wheel_vels",10)
        self.cmd_stat_pub = self.create_publisher(WheelTicks,"/wheel_ticks",10)
        self.cmd_tick_pub = self.create_publisher(WheelStatus,"/wheel_status",10)


    def send_movement_command(self,msg):
        vel = WheelVels()
        tick = WheelTicks()
        status = WheelStatus()

        #assign movement stuff here 
        vel = msg.vel
        #tick = msg.tick
        #status = msg.status

        self.cmd_vel_pub.publish(vel)
        #self.cmd_tick_pub.publish(tick)
        #self.cmd_stat_pub.publish(status)

        self.get_logger().info("Moving... ")

    def listener_callback(self,msg):
        payload = msg.data
        self.get_logger().info("Message recieved...moving " + payload)
        self.send_movement_command(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MySubscriberNode()

    #Spin means the node will keep running until killed
    rclpy.spin(node)
    
    #destroy node and shutdown ros2 communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()