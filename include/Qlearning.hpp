#pragma once

#include<iostream>

class Qlearning {
 private:
    double epsilon;
    double alpha;
    double gamma;
    double qTable[1296][3];
    std::vector<int> turtleState;
    std::vector<int> turtleAction;
    int reward;
    std::vector<int> currentState;
    std::vector<int> nextState;
    int maxEpisodeCount;

 public:
    void trainQ();
    double* returnQ();
    std::vector<int> chooseAction(std::vector<int> state);
    void assignReward();
    void updateQTable();
    void runDemo();
};
