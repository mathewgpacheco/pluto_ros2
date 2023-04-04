#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pluto_interfaces.msg import TargetMove
from pluto_interfaces.msg import SignalValues
from irobot_create_msgs.msg import HazardDetectionVector
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import WheelTicks

class CommanderNode(Node):
    def __init__(self):
        super().__init__("Commander_Node")

        #Threshold to determine when obj it too close
        self.TOO_CLOSE_THRESHOLD = 200
        
        #Various states the robot could be in
        self.WANDER = 0
        self.TURN_AROUND = 1
        self.STRAIGHT = 2
        self.AVOID = 3

        #default state
        self.current_state = self.WANDER

        #Subscribe to recieve custom detector signals
        self.detector_subscriber_ = self.create_subscription(SignalValues,
        "/detect_signals",
        self.listener_callback, 
        qos_profile_sensor_data)

        #
        self.encoder_subscriber_ = self.create_subscription(WheelTicks,
        "/wheel_encoder",
        self.encoder_callback,
        qos_profile_sensor_data)


        self.get_logger().info("Commander Node initialized")

    def listener_callback(self,msg: SignalValues):
        #do pre-condition stuff before eval
        self.eval_signals(msg)


 
    def encoder_callback(self,msg:WheelTicks):
        self.get_logger().info("Time: " + str(msg.header.stamp)+ " - L: " + str(msg.ticks_left)+ " : " "R: " + str(msg.ticks_right))

    def eval_signals(self,msg: SignalValues):
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

            leftSideObj = msg.ir_signals.readings[0].value > self.TOO_CLOSE_THRESHOLD
            leftObj = msg.ir_signals.readings[1].value > self.TOO_CLOSE_THRESHOLD
            leftAngleObj = msg.ir_signals.readings[2].value > self.TOO_CLOSE_THRESHOLD
            leftAheadObj = msg.ir_signals.readings[3].value > self.TOO_CLOSE_THRESHOLD

            rightObj = msg.ir_signals.readings[4].value> self.TOO_CLOSE_THRESHOLD
            rightAngleObj = msg.ir_signals.readings[5].value > self.TOO_CLOSE_THRESHOLD
            rightAheadObj = msg.ir_signals.readings[6].value > self.TOO_CLOSE_THRESHOLD
            



            if leftSideObj:
                self.get_logger().info("Left side object detected.")
            if leftObj:
                self.get_logger().info("Left object detected.")
            if leftAngleObj:
                self.get_logger().info("Left angle object detected.")
            if leftAheadObj:
                self.get_logger().info("Left ahead object detected.")
            if rightObj:
                self.get_logger().info("Right ahead object detected.")
            if rightAngleObj:
                self.get_logger().info("Right angle object detected.")
            if rightAheadObj:
                self.get_logger().info("Right object detected.")
            
            else:
                pass

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