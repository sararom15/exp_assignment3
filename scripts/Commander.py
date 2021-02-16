#!/usr/bin/env python

## @file Commander.py 
# @brief This node generates and sends the command. 
# 
# Details: The command is randomply choosen between play and sleep. If play is selected then it generates randomly the target positions as well. 
#

import rospy
import time
import math
import random
from std_msgs.msg import String
from geometry_msgs.msg import Point


def UserAction(): 
    while True: 

        
        ## choice of the room
        target = String() 
        target = random.choice(['Entrance','LivingRoom','Bedroom', 'Bathroom', 'Closet','Kitchen'])
        ## command 
        command = String() 
        command = "GoTo " + target 
        user = rospy.Publisher("UserCommand", String, queue_size=10) 
        user.publish(command) 

        time.sleep(20) 

if __name__ == '__main__': 
    rospy.init_node('Commander') 
    UserAction() 


        




    


