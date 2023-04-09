#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.msg import IrIntensityVector
from irobot_create_msgs.msg import WheelTicks
from irobot_create_msgs.msg import WheelVels
from geometry_msgs.msg import Twist
from pluto_interfaces.msg import TargetMove
from pluto_interfaces.msg import SignalValues
from sensor_msgs.msg import BatteryState

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class RPiNode(Node):
    def __init__(self):
        super().__init__("RPi_Node")
        self.detection_method = "bumpers"

        #publish an actual move
        self.cmd_move_pub = self.create_publisher(Twist,
        "/cmd_vel",
        qos_profile_sensor_data)

        self.target_move_subscriber_ = self.create_subscription(Twist,
        "/target_move",
        self.target_move_callback,
        qos_profile_sensor_data)
        #subscribe to moves
        #self.move_subscriber = self.create_subscription(TargetMove,
        #"/target_move",
        #self.target_move_callback,
        #10)

        #subscribe to bumpers
        #self.button_subscriber_ = self.create_subscription(HazardDetectionVector,
        #"/hazard_detection",
        #self.bumper_detector_callback,
        #qos_profile_sensor_data)

        #subscribe to ir sensors
        #self.button_subscriber_ = self.create_subscription(IrIntensityVector,
        #"/ir_intensity",
        #self.ir_detector_callback,
        #qos_profile_sensor_data)

        #Monitor power supply 
        #self.battery_subscriber_ = self.create_subscription(BatteryState,
        #"/battery_state",self.battery_callback,
        #qos_profile_sensor_data)

        #publish signals back to command
        #self.detect_publisher_ = self.create_publisher(SignalValues,
        #"/detect_signals",qos_profile_sensor_data)

        #sub to encoder ticks to forward to command
        #self.encoder_subscriber_ = self.create_subscription(WheelTicks,
        #"/wheel_ticks",self.encoder_callback,
        #qos_profile_sensor_data)

        #self.velocity_subscriber_ = self.create_publisher(WheelVels,
        #"/target_move",qos_profile_sensor_data)


        #sub to encoder ticks to forward to command
       # self.encoder_subscriber_ = self.create_subscription(WheelTicks,
        #"/wheel_ticks",self.encoder_callback,
        #qos_profile_sensor_data)
        
        #Publish images
        self.image_publisher_ = self.create_publisher(Image,"image_topic2",qos_profile_sensor_data)
        
        self.bridge = CvBridge()

        self.image_subscriber_ = self.create_subscription(Image,"image_raw",self.image_callback,
        qos_profile_sensor_data)


        self.get_logger().info("RPi Node initialized")

    def image_callback(self,msg):
        try:
            cv_image =self.bridge.imgmsg_to_cv2(msg,"bgr8")
        except CvBridgeError as e:
            print(e)
        
        try:
            #self.get_logger().info("Re-publishing an image...")
            self.image_publisher_.publish(self.bridge.cv2_to_imgmsg(cv_image,"bgr8"))
        except CvBridgeError as e:
            print(e)

    def target_move_callback(self,msg: Twist):
        move = Twist()
        #move.linear.x = msg.linear.x
        move.angular.z = msg.angular.z

        self.get_logger().info("sending create3 a msg: " + str(move))
        self.cmd_move_pub.publish(move)

    #Forward the message back to command
    def encoder_callback(self,msg:WheelTicks):
        self.get_logger().info("Inside encoder callback L/R: "+ str(msg.ticks_left) + " : " +str(msg.ticks_right))
        self.encoder_publisher_.publish(msg)


    def bumper_detector_callback(self,msg: HazardDetectionVector):
        payload = SignalValues()
        payload.bumper_signals.header.stamp = self.get_clock().now().to_msg()
        payload.detection_method = "bumpers"
        payload.bumper_signals = msg
        self.detect_publisher_.publish(payload)

    def ir_detector_callback(self,msg: IrIntensityVector):
        payload = SignalValues()
        payload.ir_signals.header.stamp = self.get_clock().now().to_msg()
        payload.detection_method = "ir"
        payload.ir_signals = msg
        self.detect_publisher_.publish(payload)
    
    def battery_callback(self,msg: BatteryState):
        self.get_logger().info("BATTERY: {:.2f}".format(100 *  msg.percentage) + "% --- TEMP: {:.2f}".format(msg.temperature)+ " celsius" +
        " --- CHARGE STATUS: " + str(msg.power_supply_status) + 
        " --- POWER SUPPLY HEALTH: " + str(msg.power_supply_health))

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