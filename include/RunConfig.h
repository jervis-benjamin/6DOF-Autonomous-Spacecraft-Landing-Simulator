/*

~ RunConfig.h ~

Owns:
- Initializing input parameters to the main sim
- Tracking whether or not we are doing Monte-Carlos (and if so how many runs)
- Generating dispersed inputs
*/

#pragma once

#include <iostream>
#include <random>

using namespace std;

class RunConfig{
private:
    // random engine
    random_device rd;
    std::mt19937 gen;

public:
    // seed random engine
    RunConfig() : gen(rd()) {}

    bool monteCarloRun = true;
    int runNum = 100; // only applies when monteCarloRun is true

    struct SimInputs {
        double initialPropMass_mult = 1.0;
        double initialPosX_add = 0.0;
        double initialPosY_add = 0.0;
        double initialPosZ_add = 0.0;
    };

    SimInputs simInputs;

    void getDispersions();
    

};