#pragma once

#include <vector>
#include <algorithm>
#include <iostream>
#include <array>

class Qlearning {
 private:
     double epsilon = 0.9;
     double alpha = 0.2;
     double gamma = 0.8;
     double epsilonDiscount = 0.9986;
     int totalEpisode = 5000;
     // double qTable[1296][3];
     std::vector <std::vector<double>> qTable;
     // std::array<std::array<double, 3>, 1296> qTable;
 public:
     Qlearning();
     void learnQ(int state, int action, int reward, double val);
     int chooseAction(int index);
     void learn(int si, int act, int rew, int nsi);
     double returnEpsilon() {return epsilon;}
     void setEpsilon(double e) {epsilon = e; }
     void storeQtable(std::string path);
     void loadQtable(std::string path);
     int demo(int index);
     void testStoreQ();
};

