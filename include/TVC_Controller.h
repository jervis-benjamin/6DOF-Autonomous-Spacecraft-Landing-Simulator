/*

~ TVC_Controller.h ~

Owns: 
- Aligning the body +X axis with a desired thrust vector via TVC gimbal deflections
- Outputting the respective body force and torque vectors applied by the TVC system

*/

#pragma once

#include <Eigen/Dense>
#include <cmath>

#include "QuaternionTools.h"
#include "Dynamics.h"
#include "Propulsion.h"
#include "Spacecraft.h"

using namespace std;

class TVC_Controller {

private:
    const Spacecraft& spacecraft;
    const Dynamics& dynamics;
    const Propulsion& propulsion;

    // controller parameters
    /*
    potential improvements:
    - pitch oscillates (very small) around setpoint
    - increase Kd to extend tvc control authority lower than 30% thrust?
    */
    double pitchKp = 2036.3;
    double pitchKd = 20000.0;
    double yawKp = 2036.3;
    double yawKd = 20000.0;
    // initial Kd determined to roughly be the quotient between initial Iyy and a constant set to 1.5
    // Solved for Kp using the damping ratio formula and letting zeta = 1
    // Overall, using a methodology based on rotational damping did wonders on tuning this system, definitely recommend
        
    // physical considerations
    const double gimbalLimit = 6.0 * (PI/180); // rad (from Apollo Design Development Documentation)
    const double gimbalRate = 0.2 * (PI/180); // rad/s (from Apollo Design Development Documentation) 
    
    // direction of body +X
    const Eigen::Vector3d bodyXdir{1.0, 0.0, 0.0};

public:

    /*
    while looking at a bottom view of the vehicle (facing head on with the engine):
    +pitch gimbal rotates counter-clockwise about the body +Y axis and deflects the engine towards the body +Z axis
    -> + pitch deflection = - pitch torque
    +yaw gimbal rotates counter-clockwise about the body +Z axis and deflects the engine towards the body -Y axis
    -> + yaw deflection = - pitch torque
    */
    double pitchDeflectionRad = 0.0; // rad
    double yawDeflectionRad = 0.0; // rad
    double lastPitchDeflectionRad = pitchDeflectionRad; // rad
    double lastYawDeflectionRad = yawDeflectionRad; // rad
    double pitchTorqueCmd = 0.0; // N-m
    double yawTorqueCmd = 0.0; // N-m

    Eigen::Vector3d actualThrustVector{0.0, 0.0, 0.0}; // N
    Eigen::Vector3d TVCtorques{0.0, 0.0, 0.0}; // N-m
    
    TVC_Controller(const Spacecraft& sc, const Dynamics& dn, const Propulsion& pr);

    void runTVC(double dt, double thrustMag, Eigen::Vector3d idealThrustDirWorld);
};