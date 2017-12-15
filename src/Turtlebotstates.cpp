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

void Turtlebotstates::setCollision() { 
    collisionStatus = false;
}

std::vector<int> Turtlebotstates::returnLaserState() {
    return laserState;
}
