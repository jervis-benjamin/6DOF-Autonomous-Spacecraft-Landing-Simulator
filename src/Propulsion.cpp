/*
~ Propulsion.cpp ~

Owns:
- Engine specs
- Mdot and mass updates
- Returns thrust and desired thrust vector direction
*/


#include <Eigen/Dense>
#include <cmath>

#include "../include/Propulsion.h"
#include "../include/Spacecraft.h"
#include "../include/Dynamics.h"
#include "../include/World.h"

using namespace std;

Propulsion::Propulsion(Spacecraft& sc, const Dynamics& dn, const World& w) : spacecraft(sc), dynamics(dn), world(w){}

Eigen::Vector3d Propulsion::runVelocityController(double dt, Eigen::Vector3d guidanceVelocity){
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

void Propulsion::update(double dt, Eigen::Vector3d guidanceVelocity){
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
            thrustEngine = lowerThrottleable*maxThrust;
        }else{
            thrustEngine = rawThrust; // if PID output is achievable, actual thrust = thrust from PID 
        }
    }else{
        thrustEngine = maxThrust;
    }

    // apply thrust multiplier (for Monte-Carlo dispersions)}
    thrustEngine *= thrustMultiplier;

    // check if there is still propellant left
    if(spacecraft.propellantMass <= 0.0){
        thrustEngine = 0.0;
    }

    // cut thrust if we landed
    if (dynamics.landed){ // engine off when contact has been made
        thrustEngine = 0;
    }

    // update throttleLevel
    throttleLevel = thrustEngine / maxThrust;

}

void Propulsion::updateMassFromEngine(double dt){
    // updates prop mass from engine thrust

    double mDot_requested = thrustEngine / v_e;
    double massToBurn = mDot_requested * dt;
    
    if (massToBurn > spacecraft.propellantMass) { // check if there even is the mass we need to burn
        // if not, burn what we have and set prop tank to 0
        thrustEngine = (spacecraft.propellantMass * v_e) / dt;
        spacecraft.propellantMass = 0.0;
    } else {
        spacecraft.propellantMass -= massToBurn;
    }

}

double Propulsion::getMaxThrust() const{
    return maxThrust;
}
