[![Build Status](https://travis-ci.org/senthilarul/QLearning-ENPM808X.svg?branch=master)](https://travis-ci.org/senthilarul/QLearning-ENPM808X)

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

# Qlearning-ENPM808X Final Project

## Overview

Autonomous robots are the next big thing influencing the tech industry. With tech majors such as google, amazon and apple investing heavily on autonomous tech it is certain that such robots a going to majorly impact our life in the near future. 

When a robot is in an unexplored environment, it does not know the set of best actions to perform for that environment in order to navigate autonomously without colliding. In this software we are taking the example of a turtlebot navigating a maze. While the Turtlebot is in the operating environment, it is nearly impossible to define all possible states, and actions for those states, prior to navigating the full environment. Even if such a case is possible, it may not generalize well for a new environment. Under these circumstances, training the agent with a sequence of rewards for the random actions that it performs, is much easier. This is where Q learning comes into play. The agent is positively rewarded when it performs a desirable actions (not colliding with obstacles), given a state. Similarly, it is negatively rewarded when it performs an undesirable action (like colliding with an obstacle). While using rewards as a learning input, it is easy to define a numerical evaluation function over all states and actions, and base the optimal policy on this evaluation function.

This project will implement a Qlerning algorithm for training a turtlebot to navigate inside a maze (created on gazebo) by avoiding obstacles. The project will show working of the algorithm by using a demo that shows the turtlebot navigating inside the maze without colliding. 

## Documents

The presentation is available at
https://docs.google.com/a/terpmail.umd.edu/presentation/d/1lY7BPjBClKzbS0OSACnfaRXx3JjSwh2sVAP6EU-dcpY/edit?usp=sharing

Full video (Presentation + Installation + Demostration of training and testing):
https://www.youtube.com/watch?v=qiqZj9iZp64


The full video has been split into presentation and demonstration:

Presentation part alone can be viewed here:
https://www.youtube.com/watch?v=k43GkDqWK9E

Demonstration part can be viewed here:
https://www.youtube.com/watch?v=CnOc-quSAEc

## SIP

For public viewing without signing in:
The product backlog file can be accessed at:
https://docs.google.com/spreadsheets/d/1vrFUu7iHakTf7QKsV2FyB1mgLVpWevoV3XNpDwdl3Bk/edit?usp=sharing

For public viewing without signing in:
The Sprint planning and review document can be accessed at:
https://docs.google.com/document/d/1hIBBBdsKGOwzNdXM68WiSKR9ENO5S5oQ6Tx5qm4As6w/edit?usp=sharing

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
## How to Build
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
rostest qlearning qlearntest.launch
```
## Running Demo
To run the demo
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch qlearning qlearn.launch
```
Gazebo opens up and terminal provides two options:
1. Train Qtable
2. Test Qtable

Please present 1 and click enter if you want to train.
The programs will ask you to enter the full path of the .csv file you want to store
the final qtable. If the .csv file doesnt exist it will be created or else it will be overwritten.

Now the gazebo simulation will show the turtlebot trying to navigate. The terminal will show
the episode count, cumulative reward and current epsilon value.
Please press ctrl + c to exit from training, the qtable (.csv) file will be saved automatically at the location you specified.

Now rerun the program and select 2 and press enter for testing the trained qtable.
The program will ask you to specify the full path of the trained qtable(.csv) file.
There are two files in this package namely "god_259.csv" and "god_1451.csv" (both available in the qtable folder). The numbers in the file names indicate the number of episodes they have been trained (higher the better). Enter the
full path of the "god_1451.csv" file and press enter. The turtlebot should navigate without colliding
and complete the maze. Press ctrl + c to exit.

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

## Doxygen
To install Doxygen
```
sudo apt install doxygen
```
once the installation is over we can generate the Doxygen documentation by running
```
doxygen ./Doxygen
```

## Dependencies
ROS Kinetic
Gazebo 7.9
turtlebot_gazebo package
RViz
Catkin

And created using C++ on a Ubuntu 16.04 LTS machine.


## Known Issues/bugs
As evident from the demo program, the maze requires the turtlebot to take only left turns. The sensor on board the turtlebot has a range of about 60 degrees. This software divides the laserscan range into 4 and uses it as the state for the Q-learning algorithm. Since the sensors visibility is towards the front the algorithm does not train well when trained in environments with both left and right turns. 
This is not a deficiency in the algorithm but in the sensor. 
A modification to a sensor with a higher angular range such as Hokuyo would solve the isse.

Sometimes the path entry for qtable during the testing doesnt work (It happens when the computer is underheavy load, for me it happens while running the program while rendering video files). Please fill in the full path in the program (qlearning class->loadQtable function instead of "path" variable)

## Coverage
Install lcov
```
sudo apt-get install lcov
```
To check coverage:
```
cd ~/catkin_ws/build
lcov --directory . --capture --output-file coverage.info
lcov --remove coverage.info '/opt/*' '/usr/*' '*/devel/*' '*test_*' '*_test*' --output-file coverage.info
lcov --list coverage.info
```
The coverage output is given as a screenshot in the results folder.
It gave a coverage of 95.9%.

## About Me

I am Senthil, a graduate student at the University of Maryland, College Park.
I major in Systems Engineering with a specialization in Robotics. I am passionate about Robotics and software development, especially in the field of Artificial Intelligence.
