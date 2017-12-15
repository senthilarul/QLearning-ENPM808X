#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include "Navigation.hpp"

class TestVel {
 private:
     geometry_msgs::Pose pose;
     geometry_msgs::Twist twist;
 public:
     void testVelocityPublish(const geometry_msgs::Twist::ConstPtr& msg) {
             twist = *msg;
     }
};

TEST(TESTNavigation, verifyCorrectStateIndex) {
  Navigation Nav;
  std::vector<int> state = {1, 1, 1, 1};
  ASSERT_EQ(0, Nav.findStateIndex(state));
}

TEST(TESTNavigation, testVelocityPublish) {
    ros::NodeHandle nh;
    TestVel test;
    Navigation Nav;
    ros::Rate loop_rate(10);
    ros::Subscriber sub = nh.subscribe("/mobile_base/commands/velocity", 1000,
                                      &TestVel::testVelocityPublish,
                                      &test);
    loop_rate.sleep();
    EXPECT_EQ(1, sub.getNumPublishers());
}
