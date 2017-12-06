#include "Navigation.hpp"
#include "std_srvs/Empty.h"

Navigation::Navigation() {

    velocity = nh.advertise <geometry_msgs::Twist>
 ("/mobile_base/commands/velocity", 1000);
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    // publish velocity values for turtlebot
    velocity.publish(msg);
}

Navigation::~Navigation() {
  // stop the robot motion
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  velocity.publish(msg);
}

void Navigation::moveTurtleBot() {

    ros::Rate loop_rate(10);
    while (ros::ok()) {
      msg.linear.x = 0.2;
      msg.angular.z = 0.0;
      velocity.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
    }

}

void Navigation::callEnvironment() {

}

void Navigation::resetEnvironment() {
    std_srvs::Empty resetWorldSrv;
    ros::service::call("/gazebo/reset_world", resetWorldSrv);
}
