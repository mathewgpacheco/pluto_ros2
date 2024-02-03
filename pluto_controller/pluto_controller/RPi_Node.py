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

        self.input_move_subscriber_ = self.create_subscription(Twist,
        "/input_move",
        self.forward_cmd_vel,
        qos_profile_sensor_data)

        self.delegate_movement_ = self.create_publisher(Twist,
        "/cmd_vel",
        qos_profile_sensor_data)

        self.get_logger().info("RPi Node initialized")


    def forward_cmd_vel(self,msg: Twist):
        self.get_logger().info('I heard ' + str(msg))
        self.delegate_movement_.publish(msg)


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