// Created by Magnus on 01/30/25

#ifndef GO1_TESTS_H
#define GO1_TESTS_H

// Package-specific header files (this will expand as we have more functions to test)
#include "go1StanceMPC.h"
#include "go1StateEstimator.h"

class go1TestFunctions {
    public:
        go1TestFunctions();
        int testZeroPosErrorGRF();
        int testNonzeroPosErrorGRF();
        int testZeroPosErrorWalk();
        int testNonzeroPosErrorWalk();
        int testRaibertHeuristic();
        int testAmirHLIP();
        int testSwingPD();
        int testBezierPos();
        int testBezierVel();
        int testNumericJacobian();

        go1State tester_state;
        mjtNum tester_joint_angles[19];
        mjtNum tester_joint_velocities[18];
};

#endif //GO1_TESTS_H