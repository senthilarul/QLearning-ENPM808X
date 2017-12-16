#pragma once
/**
 * Copyright (c) 2017, Senthil Hariharan Arul
 *
 * Redistribution and use in source and binary forms, with or without  
 * modification, are permitted provided that the following conditions are 
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in the   
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its 
 * contributors may be used to endorse or promote products derived from this 
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS 
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    Turtlebotstates.hpp
 * @author  Senthil Hariharan Arul
 * @copyright 3-clause BSD
 * @brief Turtlebotstates class declartion
 * Class to call laserscan image topic, check collision possibility and 
 * converts laserdata to states.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
// #include "Qlearning.hpp"

/**
 * @brief Class Turtlebotstates
 * The following class provides functions for read laserscan topic
 * and identifying possibility of turtlebot collision with environment.
*/

class Turtlebotstates {
 private:
  // flag to indicate presence or absense of an obstacle
  bool collisionStatus = false;
  // bool collision = false;
  std::vector<int> laserState;
  // Qlearning ql;

 public:
 /**
  * @brief constructor Turtlebotstate
  * @param none
  * @return none
  * initializes values of collision status
  */
  Turtlebotstates();

 /**
  * @brief destructor Turtlebotstates
  * @param none
  * @return none
  * destroy the DepthImage object created
  */
  ~Turtlebotstates();

 /**
  * @brief function callDepth
  * @param msg of type sensor_msgs::LaserScan
  * @return none 
  * A call back function to read laserscan data to update collision status 
  */
  void callDepth(const sensor_msgs::LaserScan::ConstPtr& msg);

 /**
  * @brief function collisionCheck
  * @param none
  * @return 1 for presence if obstacle and 0 for its absence
  * Return the current value of collisionStatus
  */
  bool collisionCheck();

 /**
  * @brief function setCollision
  * @param none
  * @return none
  * sets values for collisionStat variable
  */
  void setCollision();

 /**
  * @brief function returnLaserState
  * @param none
  * @return state as std::vector
  * Return the current state of the turtlebot
  */
  std::vector<int> returnLaserState();
};

