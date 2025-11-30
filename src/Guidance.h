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
    Spacecraft& spacecraft;


    void updateIIP(){
        double g = world.getGravitationalAccel(dynamics.position.z());
        double t = (dynamics.velocity.z() + sqrt(pow(dynamics.velocity.z(), 2.0) + 2 * dynamics.position.z() * g)) / g;
        
        iip.x() = dynamics.velocity.x() * t + dynamics.position.x();
        iip.y() = dynamics.velocity.y() * t + dynamics.position.y();
        iip.z() = 0.0;
    }

    void determinePhase(){
        //modify phase index
    }
    
    void phaseActions(){
        //initiate appropriate response based on phase index
    }


public:
    Eigen::Vector3d iip;
    const string phase[4] = {
        "Powered Descent Initation (PDI)", 
        "Throttle Recovery (TR)", 
        "Terminal Guidance 1 (TG-1)", 
        "Terminal Guidance 2 (TG-2)"
    };
    int phaseIndex;

    Guidance(const Dynamics& dyn, const World& w, Spacecraft& sc): dynamics(dyn), world(w), spacecraft(sc){
        updateIIP();

        phaseIndex = 0;
    }

    void update(){
        determinePhase();
        phaseActions();
    }


};