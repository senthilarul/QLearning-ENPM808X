#include <ros/ros.h>
#include <gtest/gtest.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "qTest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
