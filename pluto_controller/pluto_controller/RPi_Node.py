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

from std_msgs.msg import String



class RPiNode(Node):
    def __init__(self):
        super().__init__("RPi_Node")

        #subscribe to bumpers
        self.button_subscriber_ = self.create_subscription(HazardDetectionVector,
        "/hazard_detection",
        self.bumper_detector_callback,
        qos_profile_sensor_data)


        
        #test subscription 
        self.test_subscriber_ = self.create_subscription(String,
        "topic",
        self.listener_callback,
        10)


        #publish signals back to command
        self.detect_publisher_ = self.create_publisher(SignalValues,
        "/detect_signals",qos_profile_sensor_data)

        #subscribe to ir sensors
        #self.button_subscriber_ = self.create_subscription(IrIntensityVector,
        #"/ir_intensity",
        #self.ir_detector_callback,
        #qos_profile_sensor_data)

        #Monitor power supply 
        #self.battery_subscriber_ = self.create_subscription(BatteryState,
        #"/battery_state",self.battery_callback,
        #qos_profile_sensor_data)

 

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

        self.get_logger().info("RPi Node initialized")


    def listener_callback(self,msg):
        self.get_logger().info('I heard ' + msg.data)

    def bumper_detector_callback(self,msg: HazardDetectionVector):
        payload = SignalValues()
        payload.bumper_signals.header.stamp = self.get_clock().now().to_msg()
        payload.detection_method = "bumpers"
        payload.bumper_signals = msg
        self.detect_publisher_.publish(payload)

    #def ir_detector_callback(self,msg: IrIntensityVector):
        #payload = SignalValues()
        #payload.ir_signals.header.stamp = self.get_clock().now().to_msg()
        #payload.detection_method = "ir"
        #payload.ir_signals = msg
        #self.detect_publisher_.publish(payload)
    
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