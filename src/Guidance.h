/*
~ Guidance.h ~

Owns:
- Determining guidance phases based on current state
- Assigning throttle values and control gains
*/

#pragma once

#include <Eigen/Dense>
#include "Dynamics.h"
#include "Spacecraft.h"
#include "QuaternionTools.h"
#include "World.h"
using namespace std;

class Guidance{

private:
    const Dynamics& dynamics;
    const World& world;

    Eigen::Vector3d calcIIP() const{
        Eigen::Vector3d pos = dynamics.position;
        Eigen::Vector3d vel = dynamics.velocity;
    }

    

public:
    Guidance(const Dynamics& dyn, const World& w): dynamics(dyn), world(w){
        iip = Eigen::Vector3d::Zero();
    }

    void updateGuidance(){
        iip = calcIIP();
    }

};