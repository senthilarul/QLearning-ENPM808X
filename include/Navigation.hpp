#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <iostream>

class Navigation {
 private:
    ros::NodeHandle nh;
    geometry_msgs::Twist msg;
    ros::Publisher velocity;

 public:
    void moveTurtleBot();
    void callEnvironment();
    void resetEnvironment();
};
