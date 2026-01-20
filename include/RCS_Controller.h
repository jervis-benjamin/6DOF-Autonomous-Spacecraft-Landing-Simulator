/*

~ RCS_Controller.h ~

*/

// for now, rcs consumes the same fuel as the main engine
// check to see if i am handing frames correctly (for example, should a quaternion be a relative quaternion or should i get the world quaternion)

#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <array>

#include "Dynamics.h"
#include "Spacecraft.h"
#include "QuaternionTools.h"

using namespace std;

class RCS_Controller{
private:
    Spacecraft& spacecraft;
    const Dynamics& dynamics;

    // controller parameters
    Eigen::Vector3d Kp{3000.0, 4000.0, 4000.0}; // increase Kd to get faster response at the cost of more fuel expenditure and more valve chattering
    Eigen::Vector3d Ki{0.01, 0.01, 0.01};
    Eigen::Vector3d Kd{6000.0, 10000.0, 10000.0};
    Eigen::Vector3d integralError = Eigen::Vector3d::Zero();
    double iLimit = 0.5; // integral term anti-windup limit
    double deadband = 0.07 * (PI/180); //0.001; // ~0.05 degrees

    // Marquardt R‑4D thruster specs
    const double nomThrust = 490.0; // N
    const double Isp = 312.0; // s
    const double g_0 = 	9.80665; // m/s^2
    const double v_e = Isp * g_0; // m/s

    // using hysteresis when setting thruster activation torque to prevent chattering (
    // thresholds are expressed in percent of nominal torque (torqueCmd/torqueMag)
    const double upperThreshold = 0.002; // activate RCS 
    const double lowerThreshold = 0.001; // deactivate RCS

    // direction of body +X
    const Eigen::Vector3d bodyXdir{1.0, 0.0, 0.0};

public: 
    double leverArm; // m
    double torqueMag; // N-m (magnitude)

    // LM has 16 RCS thrusters but only 2 fire for a single axis
    // since there are 3 rotation axis, there are 3 sets of 2 thrusters that fire
    array<int, 3> RCS_thrusterSet = {0, 0, 0}; // 0 -> inactive, -1 -> negative torque thruster set, +1 -> positive torque thruster set 
    Eigen::Vector3d RCStorques{0.0, 0.0, 0.0}; // N-m

    RCS_Controller(Spacecraft& sc, const Dynamics& dn);

    void runRCS(double dt, Eigen::Vector3d idealThrustDirWorld, bool ignoreRoll, double rollSetpointDeg, Eigen::Vector3d desiredAttitudeEuler, bool directAttitudeControl);

    void updateMassFromRCS(double dt);
};