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

using namespace std;

class RCS_Controller{
private:
    Spacecraft& spacecraft;
    const Dynamics& dynamics;

    // controller parameters
    Eigen::Vector3d Kp{1500.0, 1500.0, 1500.0};
    Eigen::Vector3d Ki{0.0001, 0.0001, 0.0001};
    Eigen::Vector3d Kd{250, 250, 250};
    Eigen::Vector3d integralError = Eigen::Vector3d::Zero();
    double iLimit = 0.5; // integral term anti-windup limit
    double deadband = 0.005; // ~0.28 degrees

    // Marquardt R‑4D thruster specs
    const double nomThrust = 490.0; // N
    const double Isp = 312.0; // s
    const double g_0 = 	9.80665; // m/s^2
    const double v_e = Isp * g_0; // m/s

    // using hysteresis when setting thruster activation torque to prevent chattering (
    // thresholds are expressed in percent of nominal torque (torqueCmd/torqueMag)
    const double upperThreshold = 0.15; // activate RCS 
    const double lowerThreshold = 0.05; // deactivate RCS

    // direction of body +X
    const Eigen::Vector3d bodyXdir{1.0, 0.0, 0.0};

public: 
    double leverArm; // m
    double torqueMag; // N-m (magnitude)

    // LM has 16 RCS thrusters but only 2 fire for a single axis
    // since there are 3 rotation axis, there are 3 sets of 2 thrusters that fire
    array<int, 3> RCS_thrusterSet = {0, 0, 0}; // 0 -> inactive, -1 -> negative torque thruster set, +1 -> positive torque thruster set 
    Eigen::Vector3d RCStorques{0.0, 0.0, 0.0}; // N-m

    RCS_Controller(Spacecraft& sc, const Dynamics& dn) : spacecraft(sc), dynamics(dn) {
        leverArm = spacecraft.vehWidth / 2.0;
        torqueMag = leverArm * nomThrust * 2; // multipy by 2 since there are 2 thrusters that fire per axis (torque from a single thruster set)
    }

    void runRCS(double dt, Eigen::Vector4d tunningTargetQuat, bool ignoreRoll, double rollSetpointDeg){
    //void runRCS(double dt, Eigen::Vector3d idealThrustDirWorld, bool ignoreRoll, double rollSetpointDeg){
        /* for tunning, we set targetQuat equat to the target quat in main */


        // rotate the desired thrust direction into the body frame
        // Eigen::Vector3d idealThrustDirBody = QuaternionTools::rotateVector(QuaternionTools::inverse(dynamics.getWorldOrientation()), idealThrustDirWorld);

        /* forming a target quaternion by solving for the angle and rotation axis */
        /* 
        // angle component
        double cosAngle = bodyXdir.dot(idealThrustDirBody);
        cosAngle = std::clamp(cosAngle, -1.0, 1.0); // handling potential floating point errors
        double angle = acos(cosAngle); // rad

        // axis component
        Eigen::Vector3d axis = bodyXdir.cross(idealThrustDirBody);
        Eigen::Vector3d axis_hat{0.0, 0.0, 0.0};
        if (axis.norm() > 0.0){
            axis_hat = axis.normalized();
        }

        // constructing quaternion
        Eigen::Vector4d targetQuat{ cos(angle/2.0), axis_hat.x()*sin(angle/2.0), axis_hat.y()*sin(angle/2.0), axis_hat.z()*sin(angle/2.0) };
        targetQuat.normalize();

        // since the quaternion above is relative, we will need to convert it into a world mapping quaternion to be in the same frame as the vehicle orientation
        targetQuat = QuaternionTools::multiply(dynamics.getWorldOrientation(), targetQuat);
        targetQuat.normalize();

        if(!ignoreRoll){ // if we need to maintain a certain roll position, we will need to ensure that our target orientation contains that roll
            Eigen::Vector3d targetInEuler = QuaternionTools::toEulerAngles(targetQuat); // deg
            Eigen::Vector4d baseOrientation = QuaternionTools::referenceQuat;

            // rebuilting quaternion with roll setpoint 
            // FOLLOWING PITCH-YAW-ROLL
            targetQuat = QuaternionTools::rotateQuat(baseOrientation, 'y', targetInEuler[0]);
            targetQuat = QuaternionTools::rotateQuat(targetQuat, 'z', targetInEuler[1]);
            targetQuat = QuaternionTools::rotateQuat(targetQuat, 'x', rollSetpointDeg);
            targetQuat.normalize();
        }
        */

        // DELETE THIS LINE WHEN NOT TUNNING
        Eigen::Vector4d targetQuat = tunningTargetQuat;

        // DELETE THIS BLOCK WHEN NOT TUNNING
        if(!ignoreRoll){ // if we need to maintain a certain roll position, we will need to ensure that our target orientation contains that roll
            Eigen::Vector3d targetInEuler = QuaternionTools::toEulerAngles(targetQuat); // deg
            Eigen::Vector4d baseOrientation = QuaternionTools::referenceQuat;

            // rebuilting quaternion with roll setpoint 
            // FOLLOWING PITCH-YAW-ROLL
            targetQuat = QuaternionTools::rotateQuat(baseOrientation, 'y', targetInEuler[0]);
            targetQuat = QuaternionTools::rotateQuat(targetQuat, 'z', targetInEuler[1]);
            targetQuat = QuaternionTools::rotateQuat(targetQuat, 'x', rollSetpointDeg);
            targetQuat.normalize();
        }
        
        // compute quaternion error
        Eigen::Vector4d errQuat = QuaternionTools::multiply(QuaternionTools::inverse(dynamics.getWorldOrientation()), targetQuat);
        errQuat.normalize();

        // verify if this enscribes the shortest path rotation
        if (errQuat[0] < 0){
            errQuat = -errQuat; 
        }

        // proportional term
        Eigen::Vector3d axisError{errQuat[1], errQuat[2], errQuat[3]};
        axisError *= 2;

        // having a deadband near our target so we don't waste fuel correcting a small error
        for (int i = 0; i < 3; i++) {
            if (fabs(axisError[i]) < deadband){
                axisError[i] = 0.0;
            }
        }

        // integral term
        integralError += axisError * dt;
        integralError = integralError.array().cwiseMax(-iLimit).cwiseMin(iLimit);

        // derivative term (since target rate is 0, this is just the negative of angular velocity)
        Eigen::Vector3d rateError = -dynamics.angularVelocity;

        // apply PID to get torque commands
        Eigen::Vector3d torqueCmd = (Kp.array() * axisError.array()) + (Ki.array() * integralError.array()) + (Kd.array() * rateError.array());

        // ignore roll command torque to save fuel when we are not concerned about maintaining 
        // a certain roll position (should still theoretically get us to the target orientation)
        if (ignoreRoll){
            torqueCmd.x() = 0.0;
            integralError.x() = 0.0;
        }


        /* 
        At this point we have a "command torque". we will need to compare if the magnetude of these torques are within the firing thresholds.
        If they are, we will trigger the rcs on that axis and produce a torque equal to torqueMag in magnitude with the same signage as the 
        torqueCmd element. as such, we will need to:
        
        update RCS_thrusterSet and by extension, RCStorques

        using RCS_thrusterSet we should be able to find the following as well
        check:
        1) if we have landed = turn rcs off / dont fire rcs
        2) if we ran out of fuel, dont fire rcs
        2) if the mdot out * dt from the command rcs is greater than the avaiable fuel, burn using whatever fuel we have left and set propMass to 0.0
        */

        // determine which thruster sets should activate and in what direction
        for (int i = 0; i < RCS_thrusterSet.size(); i++){
            if (fabs(torqueCmd[i]/torqueMag) >= upperThreshold){ // turn thruster on if we exceed threshold
                RCS_thrusterSet[i] = copysign(1.0, torqueCmd[i]);
            }else if (fabs(torqueCmd[i]/torqueMag) <= lowerThreshold){ // turn thruster off is we go below threshold
                RCS_thrusterSet[i] = 0;
            }
        }

        // use the thruster states and torque directions established in thrusterSet to find the RCS torques
        for (int i = 0; i < RCS_thrusterSet.size(); i++){ //multiplying the thruster set by the thruster set magnetude
            RCStorques(i) = torqueMag * RCS_thrusterSet[i];
        }

        if (dynamics.landed || (spacecraft.propellantMass == 0)){
            // if we landed or there is no remaining propellant, RCS should-not/cannot produce torque
            for (int i = 0; i < RCS_thrusterSet.size(); i++){ 
                RCStorques(i) = 0.0;
                RCS_thrusterSet[i] = 0;
            }
        }
    }

    void updateMassFromRCS(double dt){
        
        // find the number of active thruster sets
        int activeSets = 0;
        for (int i = 0; i < RCS_thrusterSet.size(); i++){
            activeSets += abs(RCS_thrusterSet[i]);
        }

        // determine how much propellant mass is consumed 
        double totalThrust = (torqueMag / leverArm) * activeSets;
        double mDot_requested = totalThrust / v_e;
        double massToBurn = mDot_requested * dt;

        if (massToBurn > spacecraft.propellantMass) { // check if there even is the mass we need to burn
            // if not, use what we have and set prop tank to 0
            totalThrust = (spacecraft.propellantMass * v_e) / dt;
            spacecraft.propellantMass = 0.0;

            // calculate new torque vector given this burnout state
            double RCSburnoutTorque = (totalThrust / activeSets) * leverArm;
            for (int i = 0; i < RCS_thrusterSet.size(); i++){
                RCStorques(i) = RCSburnoutTorque * RCS_thrusterSet[i];
            }
        } else {
            spacecraft.propellantMass -= massToBurn;
        }
    }
};