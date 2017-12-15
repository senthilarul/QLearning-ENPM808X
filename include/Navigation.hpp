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
 * @file    Navigation.hpp
 * @author  Senthil Hariharan Arul
 * @copyright 3-clause BSD
 * @brief Navigation class declartion 
 * Declares functions for publishing velocity command
 */
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "Turtlebotstates.hpp"
#include "std_srvs/Empty.h"
#include "Qlearning.hpp"

/**
 * @brief Class Navigation
 * class to publish velocity
 */
class Navigation {
 private:
    ros::NodeHandle nh;
    geometry_msgs::Twist msg;
    ros::Publisher velocity;
    ros::Subscriber depthBuffer;
    Turtlebotstates depth;
    int reward;
    bool collisionStat;
    Qlearning q;

 public:
   /**
    * @brief constructor Navigation class
    * @param none
    * @return none
    * initializes values of linear and angular velocity for topic
    * and initializes the publish and subscribe variables
    */
    Navigation();

   /**
    * @brief destructor Navigation class
    * @param none
    * @return none
    * destroys the created Navigation class object when object
    * goes out of scope.
    */
    ~Navigation();

   /**
    * @brief function move
    * @param none
    * @return none
    * provide velocity commands to the turtlebot according the presence
    * and absence of obstacle.
    */
    void move();
    void navmain(std::string path);
    void navmain2(std::string path);
    int findStateIndex(std::vector<int> state);
    void envReset();
    void action(int action, bool &colStat, int &reward, int &nextState);
    void demoAction(int action);
    void envPause();
    void envUnPause();
    int returnReward() { return reward;}
};
