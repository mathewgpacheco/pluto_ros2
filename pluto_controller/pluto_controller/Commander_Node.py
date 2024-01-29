#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pluto_interfaces.msg import TargetMove
from pluto_interfaces.msg import SignalValues
from irobot_create_msgs.msg import HazardDetectionVector
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import WheelTicks
from irobot_create_msgs.msg import WheelVels
from geometry_msgs.msg import Twist
from std_msgs.msg import String


from sensor_msgs.msg import Image

class CommanderNode(Node):
    def __init__(self):
        super().__init__("Commander_Node")
        
        #Subscribe to recieve custom detector signals
        self.detector_subscriber_ = self.create_subscription(SignalValues,
        "/detect_signals",
        self.signal_callback,
        qos_profile_sensor_data)

                
        #test subscription 
        self.test_publisher_ = self.create_publisher(String,
        "topic",
        10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.get_logger().info("Commander Node initialized")
 
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World : %d' % self.i
        self.test_publisher_.publish(msg)
        self.get_logger().info("publishing: " + msg.data)
        self.i +=1

    def signal_callback(self,msg: SignalValues):

        if msg.detection_method == "bumpers":
            for signal in msg.bumper_signals.detections:
                if signal.header.frame_id == "bump_left":
                    self.get_logger().info("LEFT BUMPER at tStamp: " + str(msg.bumper_signals.header.stamp))
                if signal.header.frame_id  == "bump_front_left":
                    self.get_logger().info("LEFT FRONT BUMPER at tStamp: " + str(msg.bumper_signals.header.stamp))
                if signal.header.frame_id == "bump_right":
                    self.get_logger().info("RIGHT BUMPER at tStamp: " + str(msg.bumper_signals.header.stamp))
                if signal.header.frame_id  == "bump_front_right":
                    self.get_logger().info("RIGHT FRONT BUMPER at tStamp: " + str(msg.bumper_signals.header.stamp))


def main(args=None):
    rclpy.init(args=args)
    node = CommanderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    finally:
        node.destroy_node()
        #destroy node and shutdown ros2 communication
        rclpy.shutdown()