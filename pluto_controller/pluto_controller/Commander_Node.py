#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist

from pynput import keyboard

from sensor_msgs.msg import Image

map = {'1':'Manual','2':'Automatic','q':'Rotate left','s':'Backward','w':'Forward','e':'Rotate right'}
class CommanderNode(Node):
    def __init__(self):
        super().__init__("Commander_Node")

        self.move_publisher= self.create_publisher(Twist,"/input_move",qos_profile_sensor_data)
        self.currentMode = '1'
        self.keyboard = keyboard
        self.listener = self.keyboard.Listener(on_press=self.on_press,on_release=self.on_release)
        self.listener.start()
        self.get_logger().info("Commander Node initialized. Current mode: " + map[self.currentMode])



    def on_press(self,key):
        #msg = TargetMove()
        action = Twist()
        next_move = False
        try:
            if key.char in map.keys() and self.currentMode == '1':
                next_move = True
                key = key.char
                if key == 'w':
                    action.linear.x = 2.0
                    action.linear.z = 1.0

                elif key =='s':
                    action.linear.x = -1.0
                elif key =='q':
                    pass
                elif key =='e':
                    pass
            else:
                action.linear.x = 0.0
        except AttributeError:
            print('key {0} was pressed.'.format(key))
        if next_move==True:
                self.move_publisher.publish(action)
                self.get_logger().info("Published a command: "+str(action))
       
    def on_release(self,key):
        pass
        #print('key {0} was released.'.format(key))

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