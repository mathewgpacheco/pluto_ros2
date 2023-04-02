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
        if msg.detection_method == "bumpers":
            for signal in msg.bumper_signals.detections:
                if signal.header.frame_id == "bump_left":
                    self.get_logger().info("LEFT BUMPER")
                if signal.header.frame_id  == "bump_front_left":
                    self.get_logger().info("LEFT FRONT BUMPER")
                if signal.header.frame_id == "bump_right":
                    self.get_logger().info("RIGHT BUMPER")
                if signal.header.frame_id  == "bump_front_right":
                    self.get_logger().info("RIGHT FRONT BUMPER")
       
        if msg.detection_method =="ir":

            ir_intensity_side_left = msg.ir_signals.readings[0].value
            ir_intensity_left = msg.ir_signals.readings[1].value
            ir_intensity_front_left = msg.ir_signals.readings[2].value
            ir_intensity_front_center_left = msg.ir_signals.readings[3].value
            ir_intensity_right = msg.ir_signals.readings[4].value
            ir_intensity_front_right = msg.ir_signals.readings[5].value
            ir_intensity_front_center_right = msg.ir_signals.readings[6].value
            
            self.get_logger().info("IR readings: "+ 
            "IR   SD   LFT:  " + str(ir_intensity_side_left) + 
            "IR        LFT:  " + str(ir_intensity_left) + 
            "IR   FR   LFT:  " + str(ir_intensity_front_left) +
            "IR FR CNT LFT:  " + str(ir_intensity_front_center_left)+
            "IR        RGT:  " + str(ir_intensity_right)+ 
            "IR   FR   RGT:  " + str(ir_intensity_front_right)+
            "IR FR CNT RGHT:  " + str(ir_intensity_front_center_right))

            #can detect objects now :D

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