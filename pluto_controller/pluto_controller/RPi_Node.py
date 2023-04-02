#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.msg import IrIntensityVector
from geometry_msgs.msg import Twist
from pluto_interfaces.msg import TargetMove
from pluto_interfaces.msg import DetectSignals
from sensor_msgs.msg import BatteryState

class RPiNode(Node):
    def __init__(self):
        super().__init__("RPi_Node")
        self.detection_method = "bumpers"

        #publish an actual move
        self.cmd_move_pub = self.create_publisher(Twist,"/cmd_vel",10)

        #subscribe to bumpers
        self.button_subscriber_ = self.create_subscription(HazardDetectionVector,
        "/hazard_detection",
        self.bumper_detector_callback,
        qos_profile_sensor_data)

        #subscribe to ir sensors
        self.button_subscriber_ = self.create_subscription(IrIntensityVector,
        "/ir_intensity",
        self.ir_detector_callback,
        qos_profile_sensor_data)

        #subscribe to battery level
        self.battery_subscriber_ = self.create_subscription(BatteryState,
        "/battery_state",self.battery_callback,
        qos_profile_sensor_data)

        #publish signals back to command
        self.detect_publisher_ = self.create_publisher(DetectSignals,
        "/detect_signals",
        qos_profile_sensor_data)


        self.get_logger().info("RPi Node initialized")


    def bumper_detector_callback(self,msg: HazardDetectionVector):
        payload = DetectSignals()
        payload.bumper_signals.header.stamp = self.get_clock().now().to_msg()
        payload.detection_method = "bumpers"
        payload.bumper_signals = msg
        self.detect_publisher_.publish(payload)

    def ir_detector_callback(self,msg: IrIntensityVector):
        payload = DetectSignals()
        payload.ir_signals.header.stamp = self.get_clock().now().to_msg()
        payload.detection_method = "ir"
        payload.ir_signals = msg
        self.detect_publisher_.publish(payload)
    
    def battery_callback(self,msg: BatteryState):
        self.get_logger().info("BATTERY %: "+str(msg.percentage) + " CELL TEMP: " + str(msg.cell_temperature))

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