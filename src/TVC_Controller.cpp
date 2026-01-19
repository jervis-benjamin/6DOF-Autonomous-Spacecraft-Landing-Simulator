/*

~ TVC_Controller.cpp ~

Owns: 
- Aligning the body +X axis with a desired thrust vector via TVC gimbal deflections
- Outputting the respective body force and torque vectors applied by the TVC system

*/

#include <Eigen/Dense>
#include <cmath>

#include "../include/TVC_Controller.h"
#include "../include/QuaternionTools.h"
#include "../include/Dynamics.h"
#include "../include/Propulsion.h"
#include "../include/Spacecraft.h"

using namespace std;

TVC_Controller::TVC_Controller(const Spacecraft& sc, const Dynamics& dn, const Propulsion& pr) : spacecraft(sc), dynamics(dn), propulsion(pr){}

void TVC_Controller::runTVC(double dt, double thrustMag, Eigen::Vector3d idealThrustDirWorld){
    // calculates TVC gimbal deflections to align with a desired thrust vector, and calculates the actual thrust and torque applied by the TVC system

    // rotate the desired thrust direction into the body frame
    Eigen::Vector3d idealThrustDirBody = QuaternionTools::rotateVector(QuaternionTools::inverse(dynamics.getWorldOrientation()), idealThrustDirWorld);

    /*
    since the TVC produces engine deflections, we can represent the desired thrust vector into an axis angle vector 
    using the y and z components of this vector, we can find the corespinding pitch and yaw error gimbal angles 
    */

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

    // axis-angle error vector
    Eigen::Vector3d axisAngleError = angle * axis_hat;

    // correspond with error in gimbal 
    double pitchError = axisAngleError.y();
    //cout << pitchError << endl;
    double yawError = axisAngleError.z();

    /*
    since pitch and yaw components of thrust axis angle vector are proportional to TVC gimbal deflections, we use a P controller 
    with a D term that handles angular rate feedback (damp control effort if we are already moving in the intended direction)
    */
    pitchTorqueCmd = (pitchKp * pitchError) - (pitchKd * dynamics.angularVelocity.y());
    yawTorqueCmd = (yawKp * yawError) - (yawKd * dynamics.angularVelocity.z());

    
    // normalize control effort by thrust
    if ((thrustMag > (propulsion.getMaxThrust()*0.3)) && !dynamics.landed){ // only if TVC still has controll authority and we arent on the ground yet (TVC more unreliable below 30% max thrust)
        pitchDeflectionRad = -pitchTorqueCmd / (thrustMag * spacecraft.cg.x()); // positive deflection -> negative torque
        yawDeflectionRad = -yawTorqueCmd / (thrustMag * spacecraft.cg.x());
    }else{
        pitchDeflectionRad = 0.0;
        yawDeflectionRad = 0.0;
    }

    // clamp within deflection limits
    pitchDeflectionRad = clamp(pitchDeflectionRad, -gimbalLimit, gimbalLimit);
    yawDeflectionRad = clamp(yawDeflectionRad, -gimbalLimit, gimbalLimit);

    // correct for rates
    if (fabs(pitchDeflectionRad - lastPitchDeflectionRad) > gimbalRate*dt){
        pitchDeflectionRad = lastPitchDeflectionRad + copysign(gimbalRate*dt, (pitchDeflectionRad - lastPitchDeflectionRad));
    }
    if (fabs(yawDeflectionRad - lastYawDeflectionRad) > gimbalRate*dt){
        yawDeflectionRad = lastYawDeflectionRad + copysign(gimbalRate*dt, (yawDeflectionRad - lastYawDeflectionRad));
    }

    // update previous deflection angles
    lastPitchDeflectionRad = pitchDeflectionRad;
    lastYawDeflectionRad = yawDeflectionRad;

    // convert to thrust vector given thrust 
    /*
    equations formed by applying the rotation matrices y then z. the order of gimbal deflections 
    depend on the mechanical placement of the gimbals, but for this case, we will follow the same euler 
    sequence (pitch-yaw-roll). mechanically, this results in the "inner ring" of the TVC responsible 
    for pitch deflections while the "outer ring" of the TVC responsible for the yaw deflections
    */
    actualThrustVector.x() = thrustMag * cos(pitchDeflectionRad) * cos(yawDeflectionRad);
    actualThrustVector.y() = thrustMag * cos(pitchDeflectionRad) * sin(yawDeflectionRad);
    actualThrustVector.z() = -thrustMag * sin(pitchDeflectionRad);

    // finding torque vector produced by TVC
    // equations are formed by r.cross(thrust vector), where r is [-cg.x(), 0.0, 0.0]
    TVCtorques.x() = 0.0;
    TVCtorques.y() = spacecraft.cg.x() * actualThrustVector.z();
    TVCtorques.z() = -spacecraft.cg.x() * actualThrustVector.y();
}
