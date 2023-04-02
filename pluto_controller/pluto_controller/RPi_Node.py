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

        #subscribe to bumpers; used to change state
        self.button_subscriber_ = self.create_subscription(HazardDetectionVector,
        "/hazard_detection",
        self.detector_callback,
        qos_profile_sensor_data)

        #publish signals back to command
        self.detect_publisher_ = self.create_publisher(DetectSignals,
        "/detect_signals",
        qos_profile_sensor_data)


        self.get_logger().info("RPi Node initialized")



    def detector_callback(self,msg: HazardDetectionVector):
        payload = DetectSignals()
        payload.data = "this is my data message"
        payload.signals = msg
        self.detect_publisher_.publish(payload)
        
def main(args=None):
    rclpy.init(args=args)
    node = RPiNode()

    try:
        #weeeee
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    #destroy node and shutdown ros2 communication
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()