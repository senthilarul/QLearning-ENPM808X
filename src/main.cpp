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
 * @file    main.cpp
 * @author  Senthil Hariharan Arul
 * @copyright 3-clause BSD
 * @brief main function for running the demo
 */

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include "Qlearning.hpp"
#include "Turtlebotstates.hpp"
#include "Navigation.hpp"

/**
 * @brief    main function
 * @param    argc int
 * @param    argv char array
 * @return   0
 */
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "qlearn");
    Navigation nav;
    ROS_INFO("hi main");

    ros::Duration(1.5).sleep();

    int choice = 0;
    std::string qtablePath;
    std::cout << "Welcome to the Qlearning software" << std::endl;
    std::cout << "Please enter your choice" << std::endl;
    std::cout << "1. Train Qtable (Enter 1)" << std::endl;
    std::cout << "2. Test Qtable (Enter 2)" << std::endl;

    std::cout << "Note: Since we are training the qtable, how well it gets trained depended on the number of iterations it is run. When the turtle bot is getting trained the user can turnoff training and generated a qtable when ever he wants. For the default world used it takes about 500 episode to see a visible difference in how the turtlebot navigates. Please press ctrl + c when ever you want to stop training or testing and exit the program" << std::endl;
    std::cin >> choice;
    if (choice == 1) {
        std::cout << "Please enter the full path to store the qtable (with the.csv file name)" << std::endl;
        std::cin >> qtablePath;
        nav.navmain(qtablePath);
    } else if (choice == 2) {
        std::cout << "Please enter the full path to the qtable(.csv file)" << std::endl;
        std::cin >> qtablePath;
        nav.navmain2(qtablePath);
    } else {
        std::cout << "wrong input";
    }

    std::cout << "Bye, Have a good day!";
    return 0;
}
