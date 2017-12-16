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
 * @file    Qlearning.cpp
 * @author  Senthil Hariharan Arul
 * @copyright 3-clause BSD
 * @brief class method definitions for Qlearning class
 * performs Qlearning algorithm
 */

#include <ros/ros.h>
#include <time.h>
#include <cmath>
#include <cstdlib>
#include <random>
#include <vector>
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include "Qlearning.hpp"

Qlearning::Qlearning() {
    for (int i = 0; i < 1296; i++) {
        std::vector<double> row(3, 0.0);
        qTable.push_back(row);
    }
}

void Qlearning::learnQ(int state, int action, int reward, double val) {
    // std::cout<<"state: "<<state<<"action"<<action<<"\n";
    double currentValue =  qTable[state][action];
    if (currentValue == 0) {
        qTable[state][action] = reward;
    } else {
        qTable[state][action] = currentValue + alpha * (val - currentValue);
    }
}

int Qlearning::chooseAction(int index) {
    std::vector <double> qState;
    qState = qTable[index];
    auto maxIterator = std::max_element(std::begin(qState), std::end(qState));
    int maxIndex = std::distance(std::begin(qState), maxIterator);
    auto largest = qState[maxIndex];
    int selectedAction = 0;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    float randNum = dis(gen);

    if (randNum < epsilon) {
        auto minIterator = std::min_element(std::begin(qState), std::end(qState));
        int minIndex = std::distance(std::begin(qState), minIterator);
        auto mag = qState[maxIndex];
        if ((std::fabs(qState[minIndex])) > (std::fabs(qState[maxIndex]))) {
            mag = qState[minIndex];
        }
        for (int j = 0; j < 3; j++) {
           qState[j] += randNum*mag - 0.5*mag;
        }
        maxIterator = std::max_element(std::begin(qState), std::end(qState));
        maxIndex = std::distance(std::begin(qState), maxIterator);
        largest = qState[maxIndex];
    }

    std::vector<int> largestQ;
    for (int i = 0; i < 3; i++) {
        if (largest == qState[i])
            largestQ.push_back(i);
    }

    if (largestQ.size() > 1) {
        std::uniform_int_distribution<> disact(0, largestQ.size()-1);
        selectedAction = largestQ[disact(gen)];
    } else {
        selectedAction = largestQ[0];
    }
    largestQ.clear();
    qState.clear();
    return selectedAction;
}

void Qlearning::learn(int si, int act, int rew, int nsi) {
    std::vector<double> qNextState;
    qNextState = qTable[nsi];
    auto maxIterator = std::max_element(std::begin(qNextState), std::end(qNextState));
    int maxIndex = std::distance(std::begin(qNextState), maxIterator);
    auto largest = qNextState[maxIndex];
    qNextState.clear();
    learnQ(si, act, rew, rew + gamma*largest);
}

void Qlearning::storeQtable(std::string path) {
    // std::ofstream out("/home/viki/enpm808/src/QLearning-ENPM808X/qtable/test.csv");
    std::ofstream out(path);
    for (int i = 0; i < 1296; i++) {
        for (int j = 0; j < 3; j++)
            out << qTable[i][j] <<',';
        out << '\n';
    }
    ROS_INFO("Qtable Stored");
}



void Qlearning::loadQtable(std::string path) {
    std::vector<double> temp;
    std::vector<std::vector<double>> temp2;
    // std::string path = ros::package::getPath("qlearning");
    // std::ifstream file("/home/viki/enpm808/src/QLearning-ENPM808X/qtable/l259.csv");
    std::ifstream file(path);
    // std::ifstream MyExcelFile ("l569.csv", ios::trunc);
    std::string row, cell;

    if (file.good()) {
        ROS_INFO("File loaded");
        int rowCount = 0;
        while (std::getline(file, row)) {
            int columnCount = 0;
            std::stringstream linestream(row);
            while (getline(linestream, cell, ',')) {
                std::stringstream convertor(cell);
                convertor >> qTable[rowCount][columnCount];
                ++columnCount;
            }
            temp2.push_back(temp);
            ++rowCount;
        }
    } else {
       ROS_ERROR("file cannot be loaded");
       std::cerr << "error";
       exit(-1);
    }
}



int Qlearning::demo(int index) {
    std::vector <double> qState;
    qState = qTable[index];
    auto maxIterator = std::max_element(std::begin(qState), std::end(qState));
    int maxIndex = std::distance(std::begin(qState), maxIterator);
    return maxIndex;
}

void Qlearning::testStoreQ() {
    qTable[0][0] = 1;
    qTable[0][1] = 2;
    qTable[0][2] = 3;
}
