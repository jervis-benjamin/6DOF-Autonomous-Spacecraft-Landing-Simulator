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
    Eigen::Vector3d Kp{1.0, 1.0, 1.0};
    Eigen::Vector3d Ki{0.0001, 0.0001, 0.0001};
    Eigen::Vector3d Kd{0.5, 0.5, 0.5};

    Eigen::Vector3d integralError = Eigen::Vector3d::Zero();
    Eigen::Vector3d previousError = Eigen::Vector3d::Zero();

public:

    double throttleLevel = 0.0; // from 0-1
    double thrustEngine = maxThrust * throttleLevel; // N (actual thrust)
    Eigen::Vector3d idealthrustDirection{1.0, 0.0, 0.0}; // in the body frame

    Propulsion(Spacecraft& sc, const Dynamics& dn, const World& w) : spacecraft(sc), dynamics(dn), world(w){}

    Eigen::Vector3d runVelocityController(double dt, Eigen::Vector3d guidanceVelocity){
        // takes in desired velocity vector and outputs an resulting acceleration vector

        Eigen::Vector3d error = guidanceVelocity - dynamics.velocity;

        // update integral term, clamp to prevent wind-up, and update derivative
        integralError += error * dt;
        integralError = integralError.array().cwiseMax(-10.0).cwiseMin(10.0);
        Eigen::Vector3d derivativeError = (error - previousError) / dt;

        // run PID and update previous error
        Eigen::Vector3d idealAccel = (Kp.array() * error.array()) + (Ki.array() * integralError.array()) + (Kd.array() * derivativeError.array());
        previousError = error;

        return idealAccel; // m/s^2
    }
    
    void update(double dt, Eigen::Vector3d guidanceVelocity){
        // updates idealthrustDirection, thrustEngine, and throttleLevel

        Eigen::Vector3d idealAccel = runVelocityController(dt, guidanceVelocity);

        // since the controller outputs the net kinematic desired acceleration, we will need to seperately compute the acceration needed by the engine
        Eigen::Vector3d gravity{0.0, 0.0, -world.getGravitationalAccel(dynamics.position.z())};
        Eigen::Vector3d thrustAccel = idealAccel - gravity;

        // computing the raw thrust direction and magnitude 
        idealthrustDirection = thrustAccel.normalized();
        double rawThrust = spacecraft.totalMass * thrustAccel.norm();
        if((rawThrust <= 0.6*maxThrust) && (rawThrust >= 0.1*maxThrust)){
            inThrottleRegion = true; // not turning this false when we are out of range, this variable is only meant to be a "check" once we dip into the throttle region
        }

        // process rawThrust within physical limits
        if(inThrottleRegion){
            if (rawThrust > upperThrottleable*maxThrust){ // if we are in the throttleable region, we cannot exceed 60% thrust
                thrustEngine = upperThrottleable*maxThrust;
            }else if (rawThrust < lowerThrottleable*maxThrust){ 
                if (dynamics.landed){ // engine off when contact has been made
                    thrustEngine = 0;
                }else{
                    thrustEngine = lowerThrottleable*maxThrust;
                }
            }else{
                thrustEngine = rawThrust; // if PID output is achievable, actual thrust = thrust from PID 
            }
        }else{
            thrustEngine = maxThrust;
        }

        // check if there is still propellant left
        if(spacecraft.propellantMass <= 0.0){
            thrustEngine = 0.0;
        }

        // update throttleLevel
        throttleLevel = thrustEngine / maxThrust;

    }

    void updateMass(double dt){
        // updates prop mass, total mass, cg location, and inertia tensor

        double mDot_requested = thrustEngine / v_e;
        double massToBurn = mDot_requested * dt;
        
        if (massToBurn > spacecraft.propellantMass) { // check if there even is the mass we need to burn
            // if not, but what we have and set prop tank to 0
            thrustEngine = (spacecraft.propellantMass * v_e) / dt;
            spacecraft.propellantMass = 0.0;
        } else {
            spacecraft.propellantMass -= massToBurn;
        }

        spacecraft.totalMass = spacecraft.dryMass + spacecraft.propellantMass;
        spacecraft.updateCG();
        spacecraft.updateInertia();
    }

    double getMaxThrust() const{
        return maxThrust;
    }
};