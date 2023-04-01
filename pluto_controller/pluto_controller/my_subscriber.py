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

        self.movement_status_subscriber = self.create_subscription(WheelStatus,"/wheel_status",self.status_listener_callback,10)
        self.movement_velocity_subscriber = self.create_subscription(WheelVels,"/wheel_vels",self.status_listener_callback,10)
    def status_listener_callback(self,msg):
        status = WheelStatus()
        velocity = WheelVels()

        velocity = msg.velocity
        status = msg.status
        self.get_logger().info("Pluto wheels_enabled: " + str(status.wheels_enabled))
        self.get_logger().info("Pluto pwm left/right: " + str(status.pwm_left) + " : " + str(status.pwm_right))
        self.get_logger().info("Pluto velocity left/right " + str(velocity.velocity_left) + " : " + str(velocity.velocity_right))

    def listener_callback(self,msg):
        payload = msg.data
        move = Twist()
        move = msg.twst
        self.get_logger().info("Message recieved...moving " + str(move))
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