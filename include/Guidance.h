/*
~ Guidance.h ~

Owns:
- Scheduling velocity setpoints during various phases of descent
- Scheduling roll setpoint
*/

/*
as of now, the guidance class phase triggers are too dependant on the spacecraft's initial states. 
later on, it would be key to implement guidance that works for a large range of initial states and is more robust
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
    const Spacecraft& spacecraft;
    const Dynamics& dynamics;
    const World& world;

    /*
    landing target is at (0, 0, 0), and spacecraft position is expressed as the relative distance to target.
    that being said, in the future maybe we could add a functionality to land it any coordinate.
    */
    Eigen::Vector3d landingTarget{0.0, 0.0, 0.0};

    // guidance key triggers (find better variable names)
    /* RANGES */
    double stopBreakingBurn = 500; // m end breaking phase once we are this close to the target      
    /* ALTITUDES */
    double approachPhaseAlt = 10000; // m (altitude at which the spacecraft decends at secondDecentRate velocity)
    double lockVeclocityAlt = 1000; // m (altitude at which x and y velocity will be commanded to zero)
    double lockOrientationAlt = 800; // m (altitude at which vehicle will be oriented level with the ground)

    double breakingVelocity = -50.3; // m/s
    double approachVelocity = -38.83; // m/s 

    // descent rates
    double breakingDescentRate = -45.0; // m/s
    double approachDescentRate = -18.0; // m/s
    double lockVeclocityDescentRate = -7.0; // m/s
    double finalDescentRate = -1.4; // m/s

    // PID variables for corrective velocity controller
    double Kp = 0.2;
    double Kd = 1.8;
    double xKi = 0.1;
    double yKi = 0.0;
    double xError = 0.0;
    double yError = 0.0;

    double X_correctiveVelocity(double dt, double position, double target, double currVelocity);
    double Y_correctiveVelocity(double dt, double position, double target, double currVelocity);

    double getHorizontalDistance();

public:
    Eigen::Vector3d velocitySetpoints{0.0, 0.0, 0.0};
    bool ignoreRoll = true;
    double rollSetpointDeg = 0.0;
    bool maintainOrientation = false;
    Eigen::Vector3d eulerSetpointsDeg{0.0, 0.0, 0.0}; // final landing orientation
    double guidanceState = 0;
    
    Guidance(Spacecraft& sc, const Dynamics& dn, const World& w);

    void update(double dt);
};