/*
~ Guidance.h ~

Owns:
- Scheduling velocity setpoints during various phases of descent
- Scheduling roll setpoint
*/

// lock out translational correction after a certain altitude (keep vehicle straight up and only worry about landing)

#pragma once

#include <Eigen/Dense>
#include "Dynamics.cpp"
#include "Spacecraft.cpp"
#include "QuaternionTools.cpp"
using namespace std;

class Guidance{

private:
    const Dynamics& dynamics;
    const Spacecraft& spacecraft;

    
    
    

public:
    
    double rollSetpointDeg = 0.0;
    Eigen::Vector3d velocitySetpoints{0.0, 0.0, 0.0};
    
    Guidance(const Dynamics& dyn, const Spacecraft& sc): dynamics(dyn), spacecraft(sc){}




};