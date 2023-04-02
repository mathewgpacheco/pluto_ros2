#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetectionVector
from geometry_msgs.msg import Twist
from pluto_interfaces.msg import TargetMove
from pluto_interfaces.msg import DetectSignals

class RPiNode(Node):


    def __init__(self):
        super().__init__("RPi_Node")

        #publish an actual move
        self.cmd_move_pub = self.create_publisher(Twist,"/cmd_vel",10)

        #subscribe to interface buttons; used to change state
        self.button_subscriber_ = self.create_subscription(HazardDetectionVector,
        "/hazard_detection",
        self.bumper_callback,
        qos_profile_sensor_data)

        #publish signals back to command
        self.detect_publisher_ = self.create_publisher(DetectSignals,
        "/detect_signals",
        qos_profile_sensor_data)

        self.get_logger().info("RPi Node initialized")



    def bumper_callback(self,msg: HazardDetectionVector):
        payload = DetectSignals()
        payload.bumpers.detections = msg.detections
        for detection in msg.detections:
            print(detection.header.frame_id)
            self.detect_publisher_.publish(payload)
    #    if msg.detections[1] == 1:
    #        self.get_logger().info("Bumper pressed")
    #        self.detect_publisher_.publish(msg)
    #    else:
    #        self.get_logger().info("Something else?")
    #        self.detect_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RPiNode()

    #Spin means the node will keep running until killed
    rclpy.spin(node)
    
    #destroy node and shutdown ros2 communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()