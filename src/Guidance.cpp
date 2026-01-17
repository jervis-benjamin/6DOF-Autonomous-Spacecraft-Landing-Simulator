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
    const Spacecraft& spacecraft;
    const Dynamics& dynamics;
    const World& world;

    /*
    landing target is at (0, 0, 0), and spacecraft position is expressed as the relative distance to target.
    that being said, in the future maybe we could add a functionality to land it any coordinate.
    */
    Eigen::Vector3d landingTarget{0.0, 0.0, 0.0};

    // guidance key triggers (find better variable names)
    double stopDeorbitBurn = 200; // m (when the distance from the target to the IIP is this value, focus less on decreasing horizontal velocity)       
    double correctXrange = 50; // m (range approach distance at which the vehicle will attempt to correct for x-cor error)
    double lockVeclocityAlt = 900; // m (altitude at which x and y velocity will be commanded to zero)
    double lockOrientationAlt = 300; // m (altitude at which vehicle will be oriented level with the ground)

    // descent rates
    double initialDescentRate = -100.0; // m/s
    double secondDescentRate = -50.0; // m/s
    double thirdDescentRate = -15.0; // m/s
    double finalDescentRate = -1.4; // m/s


    double correctiveVelocity(double position, double target){
        // returns a command velocity to correct current position based on a simple function
        double cmdVelocity = -0.1 * pow((position - target), 2.0);
        return cmdVelocity;
    }

    double impactDistanceToTarget(Eigen::Vector3d velocity, Eigen::Vector3d position, double gravityMag){
        // returns the distance from the IIP to the landing target

        // determine IIP x-cor
        double IIP_x = position.x() + (velocity.x() / gravityMag) * (velocity.z() + sqrt(pow(velocity.z(), 2.0) + 2*position.z()*gravityMag));

        // determine IIP y-cor
        double IIP_y = position.y() + (velocity.y() / gravityMag) * (velocity.z() + sqrt(pow(velocity.z(), 2.0) + 2*position.z()*gravityMag));

        // determine distance from target
        double distanceToTarget = sqrt( pow((IIP_x - landingTarget.x()), 2.0) * pow((IIP_y - landingTarget.y()), 2.0) );
        return distanceToTarget;
    }



public:
    
    double rollSetpointDeg = 0.0;
    Eigen::Vector3d eulerSetpointsDeg{0.0, 0.0, 0.0}; // final landing orientation
    bool maintainOrientation = false;
    Eigen::Vector3d velocitySetpoints{0.0, 0.0, 0.0};
    
    Guidance(Spacecraft& sc, const Dynamics& dn, const World& w) : spacecraft(sc), dynamics(dn), world(w){}

    void update(){
        // reset all guidance values to default

        // update x velocity setpoint

        // update y velocity setpoint

        // update z velocity setpoint

        // update roll setpoint

        // update RCS orientation control mode boolean

        // final descent conditions
    }




};