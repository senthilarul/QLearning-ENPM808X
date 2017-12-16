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
 * @file    QlearningTest.cpp
 * @author  Senthil Hariharan Arul
 * @copyright 3-clause BSD
 * @brief Test cases for class Qlearning
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "Qlearning.hpp"

/**
 *@brief Case to test proper functioning of demo function
 *@param none
 *@return none
 */
TEST(TestQlearning1, testDemo) {
    Qlearning q;
    q.testStoreQ();
    int action = q.demo(0);
    ASSERT_EQ(2, action);
}

/**
 *@brief Case to test correct selection of action
 *@param none
 *@return none
 */
TEST(TESTQlearning, testChooseAction) {
    Qlearning q;
    q.testStoreQ();
    q.setEpsilon(-1);
    ASSERT_EQ(2, q.chooseAction(0));
}

/**
 *@brief Case to test proper initialization of Qlearning object
 *@param none
 *@return none
 */
TEST(TESTQlearning, testIntializationError) {
    EXPECT_NO_FATAL_FAILURE(Qlearning q);
}

/**
 *@brief Case to test for publishing of error code when wrong
 * path file is given
 *@param none
 *@return none
 */
TEST(TESTQlearning, testloadQtable) {
    Qlearning q;
    ASSERT_DEATH(q.loadQtable("random.csv"), "error");
}


