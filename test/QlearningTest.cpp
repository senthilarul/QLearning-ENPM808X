#include <gtest/gtest.h>
#include <ros/ros.h>
#include "Qlearning.hpp"

TEST(TestQlearning1, testDemo) {
    Qlearning q;
    q.testStoreQ();
    int action = q.demo(0);
    ASSERT_EQ(2, action);
}

TEST(TESTQlearning, testChooseAction) {
    Qlearning q;
    q.testStoreQ();
    q.setEpsilon(-1);
    ASSERT_EQ(2, q.chooseAction(0));
}

TEST(TESTQlearning, testIntializationError) {
    EXPECT_NO_FATAL_FAILURE(Qlearning q);
}

TEST(TESTQlearning, testloadQtable) {
    Qlearning q;
    ASSERT_DEATH (q.loadQtable("random.csv"), "error");
}


