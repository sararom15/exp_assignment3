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

#smach library 
import smach 
import smach_ros

#message ros 
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped 
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import CompressedImage 
from std_msgs.msg import String, Float64 
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

#actionlib
import actionlib 
import actionlib.msg 

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# msg 
from exp_assignment3.msg import BallInfoMsg

import matplotlib.pyplot as plt
import signal 
import subprocess 
import roslaunch


global rooms
global CommandForSleeping
global Finding
global Target 


## Callback for InfoBall 
InfoBall = BallInfoMsg() 
def clbk_ball(info): 
    global InfoBall
    InfoBall.detected = info.detected.data
    InfoBall.color = info.color.data
    InfoBall.radius = info.radius.data 
    InfoBall.centerx = info.centerx.data
    InfoBall.centerx = info.centery.data

## Callback for the command
usercommand = String() 
def clbk_command(data): 
    usercommand = data.data

## Callback for the position
position = Point() 
yaw = 0 
## callback for odometry 
def clbk_odometry(msg): 
    global position 
    position = msg.pose.pose.position
    ## yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


## client of actionlib server for moving the dog robot using MOVEBASE package
def move_dog(target): 

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = target[0]
    goal.target_pose.pose.position.y = target[1]

   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

   # Sends the goal to the action server.
    client.send_goal(goal)

    # cancel goal 
    while True:     
        if InfoBall.detected == True: 
            return client.cancel_goal()


   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()  







"""
## class for imformations about balls 
class Ball: 
    def __init__(self, color, lower, upper): 
        self.color = color
        self.lower = lower 
        self.upper = upper 
        self.visible = False


## initialization of balls 
balls = [   Ball('green',(50, 50, 20),(70, 255, 255)),
            Ball('yellow',(25, 50, 20),(32, 255, 255)),
            Ball('red',(0, 50, 100),(12, 255, 255)),
            Ball('blue',(105, 50, 20),(135, 255, 255)),
            Ball('pink',(143, 50, 20),(160, 255, 255)),
            Ball('black',(0, 0, 0),(179, 255, 10))]


## detection Ball 
def Detect_Ball_clbk(ros_data): 
    
    ## direct conversion to CV2 
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:
    
    ## initialization of balls 
    balls = [   Ball('green',(50, 50, 20),(70, 255, 255)),
                Ball('yellow',(25, 50, 20),(32, 255, 255)),
                Ball('red',(0, 50, 100),(12, 255, 255)),
                Ball('blue',(105, 50, 20),(135, 255, 255)),
                Ball('pink',(143, 50, 20),(160, 255, 255)),
                Ball('black',(0, 0, 0),(179, 255, 10))]
    

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
            if radius > 5: 
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                            (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                if ball.visible == False: 
                    rospy.set_param('DetectedBall',1) 
                ball.visible = True 
                rospy.loginfo_once('the %s ball is here!', ball.color)
                color = ball.color
                rospy.set_param('ColorBall', ball.color)
                vel = Twist()
                vel.angular.z = -0.002*(center[0]-400)
                vel.linear.x = -0.01*(radius-100) 
                vel_pub.publish(vel)
                if (vel.linear.x < 0.09) & (vel.angular.z < 0.09): 
                    rospy.set_param('CloseBall',1) 
            #    vel.angular.z = 0
            #    vel.linear.x = 0
            #    vel_pub.publish(vel) 
        ## the ball is not found 
    #else:

    #    vel_ = Twist()
    #    ## set an angular velocity to search around 
    #    vel_.angular.z = 0.5 
    #    vel_pub.publish(vel_) 


        ## show window 
        cv2.imshow('window', image_np)
        cv2.waitKey(2)
"""


## function with an implemented timer for sleeping
def TimeToGoToSleep(): 
    time.sleep(400) 
    rospy.set_param('CommandForSleeping',1) 


## Normal State
class Normal(smach.State): 
    
    ## inizialization
    def __init__(self):
        ## 3 outcomes defined 
        smach.State.__init__(self, outcomes=['go_to_sleep','go_to_play','track_ball'])

        self.counter = 0 

    ## execution 
    def execute(self, userdata):

        while True: 
            self.counter = self.counter + 1

            Normal = move_dog([random.randrange(-6,6), random.randrange(-6,3)])
            if Normal: 
                rospy.loginfo('Goal achieved!')

            
            if InfoBall.detected == True: 
                return 'track_ball'
             

            if rospy.get_param('CommandForSleeping') == 1: 
                return 'go_to_sleep' 


            if self.counter == 5000:
                self.counter = 0
                command = rospy.wait_for_message("UserCommand", String) 
                rospy.loginfo('it is Time to play: %s', command.data)
                res = command.data.split() 
                rospy.set_param('Target', res[1])
                rospy.loginfo('%s', res[1]) 

                return 'go_to_play' 

            


## class for information about rooms 
class Room: 
    def __init__(self, name, color, x, y): 
        self.name = name #name of the room
        self.color = color #color of the ball in the room  
        self.x = x #x-position of the room : Default is 0 
        self.y = y #y-position of the room : Default is 0


## initialization for rooms 
rooms = [   Room('LivingRoom', 'green', 0, 0),
            Room('Kitchen', 'yellow', 0, 0),
            Room('Closet', 'red',0, 0),
            Room('Entrance', 'blue',0, 0),
            Room('Bathroom','magenta',0, 0),
            Room('Bedroom', 'black', 0, 0)]

vel_pub = rospy.Publisher("/cmd_vel",
                                Twist, queue_size=1)


## Normal_Track state
class Normal_Track(smach.State): 

    ## inizialization
    def __init__(self):
        smach.State.__init__(self, outcomes=['done']) 


    ## execution 
    def execute(self, userdata):

        time.sleep(2) 
        while True: 
            rospy.loginfo('The %s ball has been detected!', InfoBall.color)
            rospy.loginfo('The centerx is %s', InfoBall.centerx)
            rospy.loginfo('The radius is %s', InfoBall.radius)
            ## follow the ball
            vel = Twist()
            vel.angular.z = 0.002*(InfoBall.centerx -400)
            vel.linear.x = -0.01*(InfoBall.radius - 100) 
            vel_pub.publish(vel)

            ## Store position 
            if (vel.linear.x < 0.09) & (vel.angular.z < 0.09): 
                rospy.loginfo('Achieved the %s ball! Lets store the position!', InfoBall.color)
                time.sleep(2) 

        for index, room in enumerate(rooms): 
            if room.color == InfoBall.color: 
                rospy.loginfo('%x', index)
                break
            else: 
                index = -1 
                rospy.loginfo('error')


        rooms[index].x = position.x 
        rooms[index].y = position.y 

        rospy.loginfo('I am in x = %x, y = %s , in the %s', rooms[index].x, rooms[index].y, rooms[index].name) 


        time.sleep(20) 
        return 'done'



## Sleep State
class Sleep(smach.State): 

    ## inizialization
    def __init__(self):
        #outcome to be back in normal state
        smach.State.__init__(self, outcomes=['sleep_to_normal'])



    ##execution 
    def execute(self, userdata): 
        rospy.loginfo('Go to sleep')
        move_dog([-5,8])

        ## sleep for a while
        time.sleep(14) 

        return 'sleep_to_normal'


## Play State 
class Play(smach.State): 

    ## inizialization
    def __init__(self):
        smach.State.__init__(self, outcomes=['play_to_normal', 'play_to_find'])


    ##execution 
    def execute(self, userdata): 

        rospy.loginfo('my target is %s', rospy.get_param('Target'))

        ## check the index of the list of the room which corresponds to our target
        for index_, room in enumerate(rooms):     
            if room.name == rospy.get_param('Target'): 
                break
            else: 
                index_ = -1 
                rospy.loginfo('Error')
        

        if (rooms[index_].x == 0) & (rooms[index_].y == 0): 
            rospy.loginfo('The room %s has not been discovered yet, lets find it', rooms[index_].name)   
            rospy.set_param('Finding',1) 
            return 'play_to_find'  
        else: 
            rospy.loginfo('I know where is %s, lets go there!', rooms[index_].name)
            play = move_dog([rooms[index_].x, rooms[index_].y])  
            if play: 
                rospy.loginfo('Achieved Target!')

                return 'play_to_normal'

## Find State
class Find(smach.State): 

    ## inizialization
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_to_play', 'track_place'])


    ##execution 
    def execute(self, userdata): 

        if rospy.get_param('Finding') == 1: 

            ## launch explore package 
            p = subprocess.Popen(["roslaunch","exp_assignment3","explore.launch"])

            while True: 
                ## if a ball has been detected 
                if InfoBall.detected == 1: 
                    ## kill the explore process 
                    p.send_signal(signal.SIGINT) 
                    time.sleep(7)
                    return 'track_place'

        if rospy.get_param('Finding') == 0: 
            return 'go_to_play'
        



## Find Track state
class Find_Track(smach.State): 
    ## inizialization
    def __init__(self):

        smach.State.__init__(self, outcomes=['found'])

    ##execution 
    def execute(self, userdata): 
        

        rospy.set_param('Finding',0)

        rospy.loginfo('The %s ball has been detected!', InfoBall.color)
        
        ## follow the ball
        vel = Twist()
        vel.angular.z = -0.002*(InfoBall.centerx-400)
        vel.linear.x = -0.01*(InfoBall.radius-100) 
        vel_pub.publish(vel)

        ## Store position 
        if (vel.linear.x < 0.09) & (vel.angular.z < 0.09): 
            rospy.loginfo('Achieved the %s ball! Lets store the position!', InfoBall.color)
            time.sleep(2) 

            for index, room in enumerate(rooms): 
                if room.color == InfoBall.color: 
                    rospy.loginfo('%x', index)
                    break
                else: 
                    index = -1 
                    rospy.loginfo('error')


            rooms[index].x = position.x 
            rooms[index].y = position.y 

            rospy.loginfo('I am in x = %x, y = %s , in the %s', rooms[index].x, rooms[index].y, rooms[index].name) 

        time.sleep(20)
        return 'found'



def main(): 

    rospy.init_node('state_machine')
    rospy.Subscriber("/odom", Odometry, clbk_odometry)
    rospy.Subscriber("/UserCommand", String, clbk_command)
    rospy.Subscriber("/InfoBalls", BallInfoMsg, clbk_ball)
    rospy.set_param('CommandForSleeping',0)
    rospy.set_param('Finding',0) 


    ## Create a main SMACH state machine

    sm_main = smach.StateMachine(outcomes=[])

    with sm_main:
        ## Add states to the container

        ## NCreate NORMAL sub-state machine 
        sm_normal = smach.StateMachine(outcomes=['normal_to_play', 'normal_to_sleep'])

        with sm_normal: 

            ## Add state to the NORMAL sub container 
            smach.StateMachine.add('NORMAL', Normal(),
                                transitions={'go_to_sleep':'normal_to_sleep',
                                              'go_to_play':'normal_to_play',
                                              'track_ball':'NORMAL_TRACK'})

            smach.StateMachine.add('NORMAL_TRACK', Normal_Track(), 
                                    transitions={'done':'NORMAL'}) 

        smach.StateMachine.add('NORMAL_CONTAINER', sm_normal, 
                                transitions={'normal_to_play':'PLAY',
                                             'normal_to_sleep':'SLEEP'})
        
        smach.StateMachine.add('SLEEP', Sleep(), 
                                transitions={'sleep_to_normal':'NORMAL_CONTAINER'})

        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'play_to_normal':'NORMAL_CONTAINER', 
                                            'play_to_find':'FIND_CONTAINER'})

        ## Create FIND sub-state machine
        sm_find = smach.StateMachine(outcomes=['find_to_play'])

        with sm_find: 

            ## Add state to the FIND sub container
            smach.StateMachine.add('FIND', Find(),
                                    transitions={'go_to_play':'find_to_play', 
                                                 'track_place':'FIND_TRACK'})
            smach.StateMachine.add('FIND_TRACK', Find_Track(), 
                                    transitions={'found':'FIND'})

        smach.StateMachine.add('FIND_CONTAINER', sm_find, 
                                transitions={'find_to_play':'PLAY'})

        

    ## Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm_main, '/SM_ROOT')
    sis.start()

    ## Execute SMACH plan
    outcome = sm_main.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
    TimeToGoToSleep() 

