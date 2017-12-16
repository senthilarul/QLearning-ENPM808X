[![Build Status](https://travis-ci.org/senthilarul/QLearning-ENPM808X.svg?branch=master)](https://travis-ci.org/senthilarul/QLearning-ENPM808X)

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

# Qlearning-ENPM808X Final Project

## Overview

Autonomous robots are the next big thing influencing the tech industry. With tech majors such as google, amazon and apple investing heavily on autonomous tech it is certain that such robots a going to majorly impact our life in the near future. 

When a robot is in an unexplored environment, it does not know the set of best actions to perform for that environment in order to navigate autonomously without colliding. In this software we are taking the example of a turtlebot navigating a maze. While the Turtlebot is in the operating environment, it is nearly impossible to define all possible states, and actions for those states, prior to navigating the full environment. Even if such a case is possible, it may not generalize well for a new environment. Under these circumstances, training the agent with a sequence of rewards for the random actions that it performs, is much easier. This is where Q learning comes into play. The agent is positively rewarded when it performs a desirable actions (not colliding with obstacles), given a state. Similarly, it is negatively rewarded when it performs an undesirable action (like colliding with an obstacle). While using rewards as a learning input, it is easy to define a numerical evaluation function over all states and actions, and base the optimal policy on this evaluation function.

This project will implement a Qlerning algorithm for training a turtlebot to navigate inside a maze (created on gazebo) by avoiding obstacles. The project will show working of the algorithm by using a demo that shows the turtlebot navigating inside the maze without colliding. 

## SIP

The product backlog file can be view using the following link:

https://docs.google.com/a/terpmail.umd.edu/spreadsheets/d/1vrFUu7iHakTf7QKsV2FyB1mgLVpWevoV3XNpDwdl3Bk/edit?usp=sharing

The Sprint planning and review document can be viewed using the following link:
https://docs.google.com/a/terpmail.umd.edu/document/d/1hIBBBdsKGOwzNdXM68WiSKR9ENO5S5oQ6Tx5qm4As6w/edit?usp=sharing

## LICENSE

```
Copyright (c) 2017, Senthil Hariharan Arul
 
Redistribution and use in source and binary forms, with or without  
modification, are permitted provided that the following conditions are 
met:
 
1. Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer.
 
2. Redistributions in binary form must reproduce the above copyright 
notice, this list of conditions and the following disclaimer in the   
documentation and/or other materials provided with the distribution.
 
3. Neither the name of the copyright holder nor the names of its 
contributors may be used to endorse or promote products derived from this 
software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS 
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
CONTRIBUTORS BE 
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
THE POSSIBILITY OF SUCH DAMAGE.

```
## Run
Follow the following steps to build the package
```
cd mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src
git clone https://github.com/senthilarul/QLearning-ENPM808X.git
cd ..
catkin_make
```

## Test
To run test
```
cd ~/catkin_ws/
catkin_make run_tests
rosrun qlearning qlearntest.launch
```
## Running Demo
To run the demo
```
cd ~/catkin_ws/
source devel/setup.bash
rosrun qlearning qlearn.launch
```
Please follow the video for running the demo and installation of package.

## Record Rosbag
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch qlearning qlearn.launch rosbagEnable:=true
```
For playing the bag file open a new terminal and type
```
roscore
```
open another terminal and type
```
cd ~/catkin_ws/src/QLearning-ENPM808X/results
rosbag play roombatopics.bag
```
## Dependencies
ROS Kinetic
Gazebo 7.9
turtlebot_gazebo package
RViz
Catkin

And created using C++ on a Ubuntu 16.04 LTS machine.

More information to be added soon.

## Known Issues/bugs
coming soon.



## About Me

I am Senthil, a graduate student at the University of Maryland, College Park.
I major in Systems Engineering with a specialization in Robotics. I am passionate about Robotics and software development, especially in the field of Artificial Intelligence.
