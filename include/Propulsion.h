/*
~ Propulsion.h ~

Owns:
- Engine specs
- Mdot and mass updates
- Returns thrust and desired thrust vector direction
*/

#pragma once

#include <Eigen/Dense>
#include <cmath>

#include "Spacecraft.h"
#include "Dynamics.h"
#include "World.h"

using namespace std;

class Propulsion {
private:
    Spacecraft& spacecraft;
    const Dynamics& dynamics;
    const World& world;

    // Apollo Lunar Lander DPS engine specs
    const double maxThrust = 47000.0; // N
    const double Isp = 311.0; // s
    const double g_0 = 	9.80665; // m/s^2
    const double v_e = Isp * g_0; // m/s

    // DPS is able to throttle from 10-60% of max thrust
    const double upperThrottleable = 0.6;
    const double lowerThrottleable = 0.1;
    bool inThrottleRegion = false; // once it hits this region, the DPS is unable to exceed 60% thrust

    // PID constants for the velocity controller
    Eigen::Vector3d Kp{0.1, 0.1, 0.1};
    Eigen::Vector3d Ki{0.0001, 0.0001, 0.0001};
    Eigen::Vector3d Kd{0.60, 0.60, 0.60};

    Eigen::Vector3d integralError = Eigen::Vector3d::Zero();
    Eigen::Vector3d previousError = Eigen::Vector3d::Zero();

    Eigen::Vector3d runVelocityController(double dt, Eigen::Vector3d guidanceVelocity);

public:

    double throttleLevel = 0.0; // from 0-1
    double thrustEngine = 0.0; // N (actual thrust)
    Eigen::Vector3d idealthrustDirection{1.0, 0.0, 0.0}; // in the world frame

    Propulsion(Spacecraft& sc, const Dynamics& dn, const World& w);
    
    void update(double dt, Eigen::Vector3d guidanceVelocity);

    void updateMassFromEngine(double dt);

    double getMaxThrust() const;
};