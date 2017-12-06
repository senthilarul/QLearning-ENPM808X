#pragma once

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>

class Turtlebotstates {
 private:
    sensor_msgs::LaserScan laserData;
    std::vector<int> states;

 public:
    void getLaserScan();
    std::vector<int> convertToStates;
    void callLaserData();
};
