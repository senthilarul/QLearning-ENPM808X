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
 * @file    TurtlebotstatesTest.cpp
 * @author  Senthil Hariharan Arul
 * @copyright 3-clause BSD
 * @brief Test cases for class Turtlebotstates
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "Turtlebotstates.hpp"

/**
 *@brief Case to test proper initialization of Turtlebotstates object
 *@param none
 *@return none
 */
TEST(TESTDepthImage, checkDepthImageInitialized) {
  Turtlebotstates depth;
  ASSERT_FALSE(depth.collisionCheck());
}

/**
 *@brief Case to test proper detection of collision
 *@param none
 *@return none
 */
TEST(TESTDepthImage, checkObstacleDetection) {
    Turtlebotstates depthData;
    ros::NodeHandle nh;
    ros::Publisher pubScan = nh.advertise<sensor_msgs::LaserScan>("/scan", 50);
    ros::Subscriber depthBuffer = nh.subscribe <sensor_msgs::LaserScan>
 ("/scan", 50, &Turtlebotstates::callDepth, &depthData);

    sensor_msgs::LaserScan laserData;
    laserData.angle_min = -0.52;
    laserData.angle_max = 0.52;
    laserData.angle_increment =  0.0016;
    laserData.time_increment = 0.0;
    laserData.range_min = 0.44;
    laserData.range_max = 10.0;
    laserData.ranges.resize(50);
    laserData.intensities.resize(50);
    for (auto& i : laserData.ranges) {
        i = 0.0;
    }
    bool col = false;
    int count = 0;
    while (ros::ok()) {
        pubScan.publish(laserData);
        if (depthData.collisionCheck()) {
            col = true;
            break;
        }
        ros::spinOnce();
        count++;
    }
    ASSERT_TRUE(col);
}
