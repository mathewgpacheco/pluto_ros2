#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pluto_interfaces.msg import TargetMove
from pluto_interfaces.msg import DetectSignals
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

    def listener_callback(self,msg):
        #do pre-condition stuff before eval
        self.get_logger().info("Listener callback commander pub")
        self.eval_signals(msg)

    def eval_signals(self,signals):
        
        for signal in signals.detections:
            if signal.header.frame_id == "bump_left":
                #service call potential state change
                self.get_logger().info("Do something.")
            if signal.header.frame_id  == "bump_right":
                self.get_logger().info("Do something else.")
            else:
                #no service call - remain in current state
                self.get_logger().info("Do nothing. msg: " + signals)
        self.get_logger().info("Detection leng: "+ str(len(signals.detections)))
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