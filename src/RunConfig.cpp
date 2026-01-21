/*

~ RunConfig.cpp ~

Owns:
- Initializing input parameters to the main sim
- Tracking whether or not we are doing Monte-Carlos (and if so how many runs)
- Generating dispersed inputs
*/

#include <iostream>
#include <random>

#include "../include/RunConfig.h"

using namespace std;

void RunConfig::getDispersions(){
    // updates the dispersed operators in SimInputs following a normal or uniform distribution

    // format
    // normal_distribution<double> dispName(mean, stdev);
    // uniform_real_distribution<double> dispName(min, max);

    // comment out the line where the simInputs variable gets initialized to skip that dispersion
    // ex. comment this out -> simInputs.initialVar_mult = initialVar_multiplier(gen;)

    uniform_real_distribution<double> initialPropMass_multiplier(0.9, 1.0);
    //simInputs.initialPropMass_mult = initialPropMass_multiplier(gen);

    normal_distribution<double> initialPosX_adder(0, 1000);
    simInputs.initialPosX_add = initialPosX_adder(gen);

    normal_distribution<double> initialPosY_adder(0, 10);
    //simInputs.initialPosY_add = initialPosY_adder(gen);
    
    normal_distribution<double> initialPosZ_adder(0, 10);
    //simInputs.initialPosZ_add = initialPosZ_adder(gen);

}