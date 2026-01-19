/*
~ Dynamics.h ~

Owns: 
- State vector
- State vector updates
*/

#pragma once

#include <Eigen/Dense>

#include "Spacecraft.h"
#include "QuaternionTools.h"
#include "World.h"

using namespace std;

class Dynamics {

private:
    const World& world;
    const Spacecraft& spacecraft;

    struct StateDerivative {
        Eigen::Vector3d positionDot;
        Eigen::Vector3d velocityDot;
        Eigen::Vector4d orientationDot;
        Eigen::Vector3d angularVelocityDot;

        StateDerivative operator+(const StateDerivative& other) const;

        StateDerivative operator*(double scalar) const;
    };


    
    StateDerivative computeDerivatives(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Vector4d& quat, const Eigen::Vector3d& angVel, const Eigen::Vector3d& bodyForce, const Eigen::Vector3d& bodyTorque) const;

    void integrateRK4(double dt, const Eigen::Vector3d& bodyForce, const Eigen::Vector3d& bodyTorque);

    void handleCollisions();


public:
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector4d orientation; // stored as a relative quaternion to the body-world quaternion of an upright spacecraft
    Eigen::Vector3d angularVelocity;

    bool landed = false;
    bool tippedOver = false; // if the landing was at too much of an angle
    
    // in Dynamics.h for usage in impact velocity
    double impactVelocity = 99999999.9; // set to NaN
    double landingTimer = 0.0; // s
    double endTimePost = 5.0; // s (controls when to end the sim after landing)

    Dynamics(const Spacecraft& sc, const World& w);
    
    Eigen::Vector4d getWorldOrientation() const;

    void update(double dt, const Eigen::Vector3d& bodyForce, const Eigen::Vector3d& bodyTorque);

};
