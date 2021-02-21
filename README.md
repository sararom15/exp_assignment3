# Assignment 3 - Experimental Robotics Laboratory 
# Introduction 
The ROS package simulates a dog robot which can move in an environment divided into 6 rooms, each room is associated to a different color ball: 

- Blue : Entrance 
- Red: Closet 
- Green Living Room 
- Yellow : Kitchen 
- Orange: Bathroom 
- Black: Bedroom 

The simulation is carried out in Gazebo and visualized in Rviz with a custom-built world. 
The robot is a differential drive robot with two wheels; also it has a camera RGB (to detect the balls) and a hokuyo laser (for the obstacle avoidance): it has been modeled with URDF. 

The possible behaviors of the robot are: normal, sleep, play, find: they are implemented using a finite state machine architecture (using SMACH). 

In Normal behavior, the robot can move in the environment, reaching random target. Having the camera always active, in case the robot detects a new ball which has not been detected before (he discovers a new room), he switches in a Track sub-state: here the robot reaches the new ball and stores his position (approximately similar to the position of the ball, since the robot is close to it). 
After a while, the robot can switch (choosing randomly) or in sleep behavior or in play behavior. 
 
As usual, in the Sleep behavior, the robot goes in the home position (x = -5, y = 8) and stays there for a while. After that, he comes back to the Normal behavior. 

In Play behavior, the robot receives a new target for the human command (GoTo + target). Therefore the robot has to obey the command: reaching the target position, and going back to the human. Since he do not know every room location, if the target room has not been discovered, he switches in Find behavior. 

In Find behavior, the robot may explore the environment (using explore-lite package); as well as in the Normal behavior, if the robot detects a new ball, he switches in a sub-state Track: here the robot reaches the new ball and store his position. When he finds the desired room (coincident with the target), he comes back to the play behavior, achieving his goal. 

The ROS version is ROS Melodic and the ROS nodes has been written in Python 2.7. To run the simulation, some packages are needed: 
vision_opencv and cv_bridge (for the vision via camera); 
gmapping package (which provides a laser based SLAM); 
move_base package (provides a local and a global planner to reach the goal avoiding the obstacles); 
explore-lite package (allows the exploration of the environment). 

# Software Architecture and State Diagrams 
The package is composed by 3 nodes, self written in Python, present in the scripts folder: 

### Commander.py 
It is the node which simulates the human command: it publishes the target (GoTo + "RoomName") in the topic “/UserCommand” every 20 secs.

### camera_processing.py
It is the node for the camera feature: it subscribes at the topic “/camera1/image_raw/compressed” to receive images from the camera and publishes in “/output/image_raw/compressed” to visualize them. The balls images are handled as well as showed using OpenCV functions. 
The informations of the balls are stored in a list (if the ball is detected or not (boolean variable), the color, the number of the mask, the radius, the x_center, if the robot is close to the ball (according to the radius) and if the detection has been the first one or not). When a ball is detected, the information of the ball are published in the topic “/InfoBalls”.
in order to send those informations to the state machine, a new msg is created (it is located in msg folder in the package): "InfoBallMsg.msg". 

### State_Machine.py
It is the main node: here the state machine is implemented: in order to perform this, the SMACH library is used. 


The node subscribes to 3 topics:
- “/InfoBalls” (receive infos about the ball, when it is detected); 
- “/UserCommand” (receive the target from the human); 
- “/odom” (receive the odometry of the robot in the map). 
Furthermore, it publishes the velocity on “/cmd_vel” when the robot has to reach the ball (in both the track sub-states). 

Three ways of enabling the robot to move in the environment were addressed: 
1. Sending a goal to the move_base action server and cancel it in case the target has been reached or in case the robot has detected the ball. 
This method is triggered in the Normal, Sleep and Play states, where the target is previously defined and the velicity could be steady. 
2. Publishing the velocity in the "/cmd_vel". 
This method is triggered in both the track sub-state, where the target is the ball, but the velocity is computed according to the distance between the robot and the ball (velocity decreases if the robot is closer to the ball). 
3. Calling the explore-lite service: it is based on the move_base action server. 
In the Find state it is used, where the robot must look for a specific ball. 

The structure of the state machine is the following: 

<p align="center"> 
<img src="https://github.com/sararom15/exp_assignment3/blob/main/images/MainStateMachine.png">
</p>

As said and explained before, the four behaviors are represented by four different states in the node: the normal and the find states are also two containers and so they have substates as well, respectively Normal_track and Find_Track. 

<p align="center"> 
<img src="https://github.com/sararom15/exp_assignment3/blob/main/images/NormalContainer.png">
</p>

<p align="center"> 
<img src="https://github.com/sararom15/exp_assignment3/blob/main/images/FindContainer.png">
</p>

1. NORMAL_CONTAINER: 
   - NORMAL: in a loop, the robot can move in the environment, sending a goal to move_base action server a random target. Whether a new ball is detected, the goal of the move_base is cancelled and it switches in NORMAL_TRACK. In case it does not detect ball, the loop continues for three times (a counter is set). After that it can switch or in Play or in Sleep, according to a random choice. 
When the play state is chosen, the node waits the command by “Commander” node, reads the target and then switches in Play. 
   - NORMAL_TRACK: if this state is activated, then a new ball has been detected. The robot start to follow the ball publishing the velocity in the topic “cmd_vel” according to the radius and center of the detection, in order to be closer to the ball (the purpose is to have a bigger radius). Once the velocity is 0, the position, taken from the odometry of the robot, is saved. The informations related to the visited rooms and unknown places are stored in a list, useful to handle all the “knowledges” of the rooms (where we have the name of the room, the associated color, the x and the y position). For default, when the room is unknown, the position is [0,0]. 

2. SLEEP: The home target position is sent to the move_base action server, and once the home is reached, the robot stays there for a while (14 secs). 

3. PLAY: First of all, a check is made about the room to be reached: if the robot already knows the position. If the robot has been already there, he knows the position, and sends that position to the move_base action server; then it goes back to the human position (the same as the home position). Otherwise, if the position is unknown, it switches in Find state, setting a parameter “Finding” equal to 1. 
The parameter is needed because the exploration must be done yet. 


4. FIND_CONTAINER:
   - FIND: here, if the parameter “Finding” is 1, the explore-lite package is launched in order to explore the unknown environment. When a new 	ball is detected, it switches in the sub-state find_track. 

   - FIND_TRACK: the robot behaves as well as in the Normal_Track. Once the new position has been stored, the parameter “Finding” is set 	to 0, which means that the exploration has been done and a ball has been found.

It returns in FIND state, which returns to PLAY, since the parameter “Finding” is 0. 

If the found ball coincides with the target, everything is fine. Otherwise the parameter “Finding” is set again in 1 and the loop continues until the wanted room will be reached. 



##Additional nodes used 
### slam_gmapping
The node is contained in the Gmapping package, which must be launched before the simulation. It provides laser-based SLAM (Simultaneous Localization and Mapping): it creates a 2D occupancy grid map from lased and pose data collected by a mobile robot. 
In order to make a map, a robot with a laser scan and the odometry data are needed. 
More about Gmapping [here](http://wiki.ros.org/gmapping).

### move_base 
The node is part of ROS navigation stack. It provides an implementation of an action (actionlib), that given a goal, will attempt to reach it. The move_base links together a global and local planner. 
a) There are three default global planners: carrot planner, navfn and global planner. The first one in complicated indoor environment, is not very practical. The second one (used by default, but it can be changes of course) uses Dijstra's algorithm to find a global path with minimum cost between the start point and the goal. The third one has more options than navfn and supports A* algorithm. 
b) Possible local planners are dwa local planner, eband local planner and teb local planner. They change according to different algorithms to generate velocity. 
More about move_base [here](http://wiki.ros.org/move_base).

### explore-lite 
The node performs frontier based exploration of the world. For this purpose, it must be launched together with the move_base as the node sends commands to the server for allowing greedy exploration.
It does not create its own costmap, which makes it easier to configure. 
More about explore-lite [here](http://wiki.ros.org/explore_lite). 

## ROS Parameters and messages
### ROS Parameters 

### ROS messages
The following ROS message is defined: 
- InfoBallMsg.msg: it contains the informations of the ball which has been detected.
The structure is: 

```
std_msgs/Bool detected
std_msgs/String color 
std_msgs/Float64 radius 
std_msgs/Float64 centerx 
std_msgs/Float64 closeball
std_msgs/Float64 firstdetection

```


##The robot 
Since no robot is given in the initial simulation, few words about it need to be spent. 
Although the structure remains similar to the one in the previous assignment, with some modifications: we have a differential drive robot, with two wheels;  a neck (fixed with the link chassis); a head, which is fixed on the neck in this case: so the revolute joint has been replaced by a fixed joint; a RGB camera, fixed on the head; in addition a hokuyo laser scan has been fixed on the link chassis (needed for slam_mapping). 
Here the URDF graph is showed. 

<p align="center"> 
<img src="https://github.com/sararom15/exp_assignment3/blob/main/images/RobotURDF.png">
</p>



