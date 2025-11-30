/*
~ Propulsion.h ~

Owns:
- Engine specs
- Mdot and mass updates
- Returns thrust from engine
*/
#pragma once

#include <Eigen/Dense>
#include <cmath>
#include "Spacecraft.h"
#include "Dynamics.h"
using namespace std;

class Propulsion {
private:
    Spacecraft& spacecraft;
    const Dynamics& dynamics;

    // Apollo Lunar Lander DPS engine specs
    const double maxThrust = 47000.0; // N
    const double Isp = 311.0; // s
    const double g_0 = 	9.80665; // m/s^2
    const double v_e = Isp * g_0; // m/s
    // DPS is able to throttle from 10-60% of max thrust
    const double upperThrottleable = 0.6;
    const double lowerThrottleable = 0.1;

public:

    double throttleLevel; // percent (from 0-1)
    bool maintainDescentRate;
    double thrustEngine; // N

    double Kp, Ki, Kd; // PID throttle controller gains
    double previousError;
    double integralError;

    Propulsion(Spacecraft& sc, const Dynamics& dn) : spacecraft(sc), dynamics(dn){
        throttleLevel = 0.0;
        thrustEngine = maxThrust * throttleLevel;
        maintainDescentRate = false;

        Kp = 4.0;
        Ki = 1.0;
        Kd = 6.0;
        previousError = 0.0;
        integralError = 0.0;
    }

    double controlledThrottle(double dt){
        // PID controller which takes in a descent rate and outputs a throttle level
        double error = spacecraft.targetDescentRate - dynamics.velocity.z();
        
        // update integral term and clamp to prevent wind-up
        integralError += error * dt;
        integralError = std::clamp(integralError, -10.0, 10.0);

        // PID output and clamping to stay within throttle region
        double throttleCommand = (Kp * error) + (Ki * integralError) + (Kd * (error - previousError));
        throttleCommand = std::clamp(throttleCommand, lowerThrottleable, upperThrottleable);

        previousError = error;

        return throttleCommand;
    }

    double update(double dt, double guidanceThrottle){ // returns thrust 
        
        if(maintainDescentRate){
            throttleLevel = controlledThrottle(dt);
        }else{
            throttleLevel = guidanceThrottle;
        }

        if(dynamics.landed){ // cut thrust once landed
            throttleLevel = 0.0;
        }
        if(spacecraft.propellantMass <= 0.0){ // cut thrust once prop has run out
            throttleLevel = 0.0;
            spacecraft.propellantMass = 0.0;
        }
        
        // update thrust and mass
        thrustEngine = maxThrust * throttleLevel;
        double mDot_out = thrustEngine / v_e;
        
        spacecraft.propellantMass -= mDot_out * dt;
        spacecraft.totalMass = spacecraft.dryMass + spacecraft.propellantMass;

        spacecraft.updateCG();
        spacecraft.updateInertia();

        return thrustEngine;
    }

};