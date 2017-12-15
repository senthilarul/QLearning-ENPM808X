#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include "Qlearning.hpp"
#include "Turtlebotstates.hpp"
#include "Navigation.hpp"


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "qlearn");
    Navigation nav;
    ROS_INFO("hi main");

    ros::Duration(1.5).sleep();

    int choice = 0;
    std::string qtablePath;
    std::cout << "Welcome to the Qlearning software"<<std::endl;
    std::cout << "Please enter your choice" << std::endl;
    std::cout << "1. Train Qtable (Enter 1)" << std::endl;
    std::cout << "2. Test Qtable (Enter 2)" << std::endl;

    std::cout << "Note: Since we are training the qtable, how well it gets trained depended on the number of iterations it is run. When the turtle bot is getting trained the user can turnoff training and generated a qtable when ever he wants. For the default world used it takes about 500 episode to see a visible difference in how the turtlebot navigates. Please press ctrl + c when ever you want to stop training or testing and exit the program" << std::endl; 
    std::cin >> choice;
    if (choice == 1) {
        std::cout <<"Please enter the full path to store the qtable (with the.csv file name)"<<std::endl;
        std::cin >> qtablePath;
        nav.navmain(qtablePath);
    } else if (choice == 2){
        std::cout <<"Please enter the full path to the qtable(.csv file)"<<std::endl;
        std::cin >> qtablePath;
        nav.navmain2(qtablePath);
    } else {
        std::cout <<"wrong input"; 
    }

    std::cout << "Bye, Have a good day!";
    return 0;
}
