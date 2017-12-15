#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "Turtlebotstates.hpp"

TEST(TESTDepthImage, checkDepthImageInitialized) {
  Turtlebotstates depth;
  ASSERT_FALSE(depth.collisionCheck());
}

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
    while(ros::ok()) {
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
