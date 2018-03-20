# The-Scarecrow
A ROBOTIC DRONE TO GUIDE AWAY THE STRAY ANIMALS FROM THE AGRICULTURAL FIELDS

INTRODUCTION
=============
The problem of stray animals is a major issue for farmers around the country .
A lot of time and energy of farmers is consumed to  distract away these stray animals like cows , dogs etc.
Crops are destroyed by these animals which leads to financial issues for farmers. 
Our project aims to automatize this task with the help of a robotic drone.


TECHNOLOGY USED
================
QUADCOPTER DRONE
CAMERA AT A HEIGHT TO MARK ARENA 
MARKER 0 FOR UNIDENTIFIED OBJECT.
COPTER WILL BE USED AS ANOTHER MARKER.
ROS(ROBOTICS OPERATING SYSTEM).
UBUNTU
PYTHON

SYSTEM SETUP
=============
A camera will be placed at the top of field to be protected. 
A marker will be used to identify as an stray animal.
The other marker will be for the drone over the field.
The drone marker will be differentiated from the animal by checking the Z-coordinate of the marker because the drone will be in  the air and its z-coordinate value will not be zero.

WORKING
========
As soon as an another marker will be placed inside the field ,the drone will chase the marker to its position. 
If the marker/stray animal changes its position then drone will again follow the  animal and reach to its position.
At last if the animal goes out of the field the drone will not follow it and will stay at its position over the field.
The animal can be forced out of the field by producing loud sound or emitting frequencies of particular value. e.g. 2.5MHz for stray dogs.   

TO STUDY CODE:
Go to subfolders launch and scripts... 
a) test.launch and hackathon.py for simulation
b) task_3.launch and realtime_hack.py for real time drone control with the above explained setup                         
