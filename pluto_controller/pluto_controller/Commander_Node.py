#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pluto_interfaces.msg import TargetMove
from pluto_interfaces.msg import SignalValues
from irobot_create_msgs.msg import HazardDetectionVector
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import WheelTicks
from irobot_create_msgs.msg import WheelVels
from geometry_msgs.msg import Twist
import random
import cv2
import mediapipe as mp

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

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

        self.LEFT = 0
        self.RIGHT = 1
        self.NONE = 2
        self.FULL_SPEED = 3
        
        self.current_mode = self.WANDER 
        self.turn_count = 0
        self.turn_direction = self.NONE
        #Subscribe to recieve custom detector signals
        #self.detector_subscriber_ = self.create_subscription(SignalValues,
        #"/detect_signals",
        #self.listener_callback, 
        #qos_profile_sensor_data)

        #
        #self.encoder_subscriber_ = self.create_subscription(WheelTicks,
        #"/wheel_encoder",
        #self.encoder_callback,
        #qos_profile_sensor_data)

        #self.cmd_move_publisher_ = self.create_publisher(WheelVels,
        #"/target_move",
        #qos_profile_sensor_data)
        
        self.bridge = CvBridge()
        
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        self.mpDraw = mp.solutions.drawing_utils
        self.IMAGE_CENTER = 320
        self.EPSILON = 0.01
        self.image_subscriber_ = self.create_subscription(Image,"image_topic2",self.image_callback,
        qos_profile_sensor_data)

        self.target_move_publisher_ = self.create_publisher(Twist,
        "/target_move",
        10)

        self.get_logger().info("Commander Node initialized")

    def image_callback(self,msg):
        try:
            cv_image =self.bridge.imgmsg_to_cv2(msg,"bgr8")
        except CvBridgeError as e:
            print(e)
        cv_image_flipped = cv2.flip(cv_image,1)
        frame_rgb = cv2.cvtColor(cv_image_flipped ,cv2.COLOR_BGR2RGB)
        res = self.hands.process(frame_rgb)

        if res.multi_hand_landmarks:
            for handLms in res.multi_hand_landmarks:
                for id, lm in enumerate(handLms.landmark):
                    h,w,c = cv_image_flipped.shape
                    cx,cy = int(lm.x * w), int(lm.y * h)

                    if id == 8:
                        cv2.circle(cv_image_flipped , (cx, cy), 10, (255,0,255), cv2.FILLED)

                        #self.get_logger().info("index: " + str(cx) + " : " + str(cy))
                        
                        #width is 640, height is 480
                        #self.get_logger().info("pic dimensions: " + str(w) + " : " + str(h))
                        self.center_robot(cx)
                self.mpDraw.draw_landmarks(cv_image_flipped ,handLms,self.mpHands.HAND_CONNECTIONS)
        cv2.imshow("Image Window", cv_image_flipped )
        cv2.waitKey(1)


        
    def center_robot(self,x):
        move = Twist()
        #move.linear.x = 0.0
        #center is 320
        # partition 0 to 300 
        # 340 to 640
        if x < (self.IMAGE_CENTER - 20):
            self.get_logger().info("left side")
            move.angular.z = -2.0
            
        if x > self.IMAGE_CENTER + 20:
            self.get_logger().info("right side")
            move.angular.z = 2.0


        if x <= self.IMAGE_CENTER +20 and x > self.IMAGE_CENTER - 20:
            self.get_logger().info("center")
            move.angular.z = 0.0
        #self.get_logger().info("publishing a twist: " + str(move))
        self.target_move_publisher_.publish(move)
        




    def listener_callback(self,msg: SignalValues):
        #do pre-condition stuff before eval
        self.eval_signals(msg)


 
    def encoder_callback(self,msg:WheelTicks):
        self.get_logger().info("Time: " + str(msg.header.stamp)+ " - L: " + str(msg.ticks_left)+ " : " "R: " + str(msg.ticks_right))

    def eval_signals(self,msg: SignalValues):

        self.get_logger().info("CURRENT MODE: " + self.current_mode)

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
            

            if self.current_mode == self.WANDER:

                #if obj ahead, avoid
                if leftAheadObj or rightAheadObj:
                    self.current_mode = self.AVOID
            
            if self.current_mode == self.AVOID:

                #if area clear, we can move around
                if not rightAngleObj and not rightAheadObj and not leftAheadObj and not leftAngleObj:
                    self.current_mode = self.WANDER
                    self.turn_direction = self.NONE

                #turn at random
                if self.turn_direction == self.NONE:
                    rnd =  random.randrange(2)
                    if rnd == 1:
                        self.turn_direction = self.LEFT
                    else:
                        self.turn_direction = self.RIGHT
        
            left_speed = self.FULL_SPEED
            right_speed =self.FULL_SPEED
            
            if self.current_mode == self.WANDER:
                if self.turn_count > 0:
                    
                    #turn left by decrementing left speed
                    if self.turn_direction == self.LEFT:
                        self.left_speed -= 2
                    else:

                        #else eright
                        self.right_speed -= 2
                    self.turn_count -= 1
                    if self.turn_count - 1 == 0:
                        self.turn_direction = self.NONE
                    else:
                        if random.random() < 20:
                            self.turn_direction = random.randrange(2)
                            self.turn_count = random.random() * 51 + 25

            if self.current_mode == self.AVOID:
                self.turn_count = 0
                if self.turn_direction ==self.LEFT:
                    left_speed = -0.5 * self.FULL_SPEED
                    right_speed = 0.5 * self.FULL_SPEED
                elif self.turn_direction ==self.RIGHT:
                    left_speed = 0.5 * self.FULL_SPEED
                    right_speed = -0.5 * self.FULL_SPEED

            next_move = TargetMove()
            next_move.vels.velocity_left = left_speed
            next_move.vels.velocty_right = right_speed
            self.cmd_move_publisher_.publish(next_move)



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