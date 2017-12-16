#pragma once
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
 * @file    Qlearning.hpp
 * @author  Senthil Hariharan Arul
 * @copyright 3-clause BSD
 * @brief Qlearning class declartion
 * performs Qlearning algorithm and stores Qtable
 */
#include <vector>
#include <algorithm>
#include <iostream>
#include <array>
#include <string>

/**
 * @brief Class Qlearning
 * class to perform qlearning algorithm, store and load qtable
 */
class Qlearning {
 private:
     double epsilon = 0.9;
     double alpha = 0.2;
     double gamma = 0.8;
     // double epsilonDiscount = 0.9986;
     // int totalEpisode = 5000;
     // double qTable[1296][3];
     std::vector <std::vector<double>> qTable;
     // std::array<std::array<double, 3>, 1296> qTable;
 public:
   /**
    * @brief constructor Qlearning class
    * @param none
    * @return none
    * initializes values for qTable
    */
     Qlearning();

   /**
    * @brief function learnQ
    * @param state as int
    * @param action as int
    * @param reward as int
    * @param val as double
    * @return none
    * updates qtable
    */
     void learnQ(int state, int action, int reward, double val);

   /**
    * @brief function chooseAction
    * @param index as int
    * @return action as int
    * selects the action to be performed based on the epsilon values
    * and qtable 
    */
     int chooseAction(int index);

   /**
    * @brief function learn
    * @param si as int
    * @param act as int
    * @param rew as int
    * @param nsi as int
    * @return none
    * identifys the max qvalue action of the next state
    */
     void learn(int si, int act, int rew, int nsi);

   /**
    * @brief function returnEpsilon
    * @param none
    * @return epsilon as double 
    * returns the current values of epsilon
    */
     double returnEpsilon() {return epsilon;}

   /**
    * @brief function setEpsilon
    * @param e as double
    * @return none
    * sets a value for epsilon
    */
     void setEpsilon(double e) {epsilon = e; }

   /**
    * @brief function storeQtable
    * @param path as std::string
    * @return none
    * store the qtable to a .csv file
    */
     void storeQtable(std::string path);

   /**
    * @brief function loadQtable
    * @param path as std::string
    * @return none
    * loads the qtable from path to the 2d vector qTable
    */
     void loadQtable(std::string path);

   /**
    * @brief function demo
    * @param index as int
    * @return action as int
    * return the best action for the state from the trained qTable
    */
     int demo(int index);

   /**
    * @brief function testStoreQ
    * @param none
    * @return none
    * function to aid testing of Qlearning class
    */
     void testStoreQ();
};

