#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pluto_interfaces.msg import TargetMove
from pluto_interfaces.msg import DetectSignals
from irobot_create_msgs.msg import HazardDetectionVector
from rclpy.qos import qos_profile_sensor_data

class CommanderNode(Node):
    def __init__(self):
        super().__init__("Commander_Node")
        
        #Subscribe to recieve custom detector signals
        self.detector_subscriber_ = self.create_subscription(DetectSignals,
        "/detect_signals",
        self.listener_callback, 
        qos_profile_sensor_data)


        self.get_logger().info("Commander Node initialized")

    def listener_callback(self,msg: DetectSignals):
        #do pre-condition stuff before eval
        self.eval_signals(msg)

    def eval_signals(self,msg: DetectSignals):
 
        for signal in msg.signals.detections:
            if signal.header.frame_id == "bump_left":
                #service call potential state change
                self.get_logger().info("LEFT BUMPER")
            if signal.header.frame_id  == "bump_front_left":
                self.get_logger().info("LEFT FRONT BUMPER")
            if signal.header.frame_id == "bump_right":
                #service call potential state change
                self.get_logger().info("RIGHT BUMER")
            if signal.header.frame_id  == "bump_front_right":
                self.get_logger().info("RIGHT FRONT BUMPER")

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