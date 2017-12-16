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
 * @file    Turtlebotstates.cpp
 * @author  Senthil Hariharan Arul
 * @copyright 3-clause BSD
 * @brief class method definition for the Turtlebotstates class
 * receives laserscan data and converts to states
 */

#include <iostream>
#include <cmath>
#include <cfloat>
#include "Turtlebotstates.hpp"

Turtlebotstates::Turtlebotstates() {
    collisionStatus = false;
}

Turtlebotstates::~Turtlebotstates() {
}

void Turtlebotstates::callDepth(const sensor_msgs::LaserScan::ConstPtr& msg) {
    std::vector<int> tempLaserState;
    float min_range = 0.6;
    int mod = (msg->ranges.size()/4);
    collisionStatus = false;
    for (int i = 0; i < msg->ranges.size(); i++) {
        if (i%mod == 0) {
            if (std::isnan(msg->ranges[i])) {
                tempLaserState.push_back(6);
            } else {
                tempLaserState.push_back(round(msg->ranges[i]));
            }
        }
        if (msg->ranges[i] < min_range) {
            collisionStatus = true;  // true indicates presence of obstacle
        }
    }
    laserState = tempLaserState;
    tempLaserState.clear();
}

bool Turtlebotstates::collisionCheck() {
    return collisionStatus;
}


std::vector<int> Turtlebotstates::returnLaserState() {
    return laserState;
}
