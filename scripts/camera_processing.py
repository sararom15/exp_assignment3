#!/usr/bin/env python

## @file state_machine.py 
# @brief This node implements a state machine which permits to move around and to search for a ball, go to sleep, and play with the ball when the last one is found. 
# 


#ros + python library 
import rospy
import roslib
import time
import math
import random
import sys 

#message ros 
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped 
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import CompressedImage 
from std_msgs.msg import String, Float64, Bool 
from tf import transformations 
from gazebo_msgs.msg import LinkState 
from nav_msgs.msg import Odometry 


#import numpy and scipy 
import numpy as np 
from scipy.ndimage import filters 

import imutils

#Open CV 
import cv2 
from sensor_msgs.msg import CompressedImage 

#msg 
from exp_assignment3.msg import BallInfoMsg


global balls 

## class for imformations about balls 
class Ball: 
    def __init__(self, color, lower, upper, radius, centerx, closeball, firstdetection ): 
        self.color = color
        self.lower = lower 
        self.upper = upper 
        self.detected = False
        self.radius = 0.0 
        self.centerx = 0 
        self.closeball = 0 
        self.firstdetection = 0 

## initialization of balls 
balls = [   Ball('green',(50, 50, 20),(70, 255, 255), 0.0, 0, 0, 0),
            Ball('yellow',(25, 50, 20),(32, 255, 255), 0.0, 0, 0, 0),
            Ball('red',(0, 50, 100),(12, 255, 255), 0.0, 0, 0, 0),
            Ball('blue',(105, 50, 20),(135, 255, 255), 0.0, 0, 0, 0),
            Ball('magenta',(143, 50, 20),(160, 255, 255), 0.0, 0, 0, 0),
            Ball('black',(0, 0, 0),(179, 255, 10), 0.0, 0, 0, 0)]      

class image_feature: 
    def __init__(self): 
        self.image_pub = rospy.Publisher("/output/image_raw/compressed", 
                                            CompressedImage, queue_size=1) 

        self.infosBall_pub = rospy.Publisher("/InfoBalls", BallInfoMsg, queue_size=1) 

        # Subscribed Topic
        self.subscriber = rospy.Subscriber(
            "/camera1/image_raw/compressed", CompressedImage, self.Det_Ball_clbk,  queue_size=1)



        
    def Det_Ball_clbk(self, ros_data): 

        ## direct conversion to CV2 
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        ## Compute the center and the radius if the ball is visible 
        for ball in balls: 
            mask = cv2.inRange(hsv, ball.lower, ball.upper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
        
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)

            cnts = imutils.grab_contours(cnts)
            center = None

            ## only proceed if at least one contour was found
            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                ## only proceed if the radius meets a minimum size
                if radius > 5 and radius < 50: 
                    cv2.circle(image_np, (int(x), int(y)), int(radius),
                                    (0, 255, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    
                    ## if the ball is known already 
                    if ball.detected == True: 
                        ball.firstdetection = 0
                        ball.radius = radius 
                        ball.centerx = center[0]
                        ball.closeball = -1 
                    else: 
                        ball.detected = True
                        ball.radius = radius 
                        ball.centerx = center[0] 
                        ball.closeball = -1 
                        ball.firstdetection = 1


                elif radius >= 50: 
                    if ball.detected == True: 
                        ball.firstdetection = 0
                        ball.radius = radius 
                        ball.centerx = center[0]
                        ball.closeball = 1
                    else: 
                        ball.detected = True
                        ball.radius = radius
                        ball.closeball = 1 
                        ball.firstdetection = 1

                else: 
                    ball.detected = False 
                    ball.radius = 0 
                    ball.centerx = 0 
                    ball.closeball = 0 
                    ball.firstdetection = 0 



                message = BallInfoMsg() 
                message.detected = Bool(ball.detected)
                message.color = String(ball.color) 
                message.radius = Float64(ball.radius)
                message.centerx = Float64(ball.centerx)
                message.closeball = Float64(ball.closeball)
                message.firstdetection = Float64(ball.firstdetection)
                self.infosBall_pub.publish(message)


            cv2.imshow('window', image_np) 
            cv2.waitKey(2) 
def main(): 
    rospy.init_node('camera_processing', anonymous = True) 



    
    image_feature() 
    rospy.spin() 
    pass


if __name__ == '__main__': 
    main() 
