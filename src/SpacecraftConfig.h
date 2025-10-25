#pragma once

#include <Eigen/Dense>
#include <cmath>
#include "QuaternionTools.h"
using namespace std;

struct SpacecraftConfig {
    double mass; // kg
    Eigen::Matrix3d inertiaTensor; // kg*m^2

    Eigen::Vector3d initialPosition; // m
    Eigen::Vector3d initialVelocity; // m/s
    Eigen::Vector4d initialOrientation; // quaternion (w, x, y, z)
    Eigen::Vector3d initialAngularVelocity; // rad/s

    SpacecraftConfig() { 
        mass = 1000.0;
        inertiaTensor = Eigen::Matrix3d::Zero();
        inertiaTensor(0,0) = 300.0;
        inertiaTensor(1,1) = 700.0;
        inertiaTensor(2,2) = 700.0;

        /*
        initialPosition << 350000, -5000, 15000; // 350km away from target with a 5km skew, comming in at 15km of alt (scaled roughly Apollo landing trajectory in a flat world)
        initialVelocity << 1700, -5, 0; // approximately 1700 m/s horizontal velocity (with a small lateral drift) and close to 0 horizontal velocity (rougly conditions close to the begining of powered decent during Apollo)
        */
        initialPosition << 0, 0, 0; 
        initialVelocity << 0, 0, 0; 
        initialAngularVelocity << 0.0, 0.0, 0.0;

        // degrees
        double initRoll = 0.0; 
        double initPitch = 0.0;
        double initYaw = 0.0;

        // defining a base upright orientation 
        Eigen::Matrix3d R; // when upright:
        R.col(0) << 0.0, 0.0, 1.0; // body +x points to inertial +z (nose of spacecraft)
        R.col(1) << 1.0, 0.0, 0.0; // body +y points to inertial +x (spacecraft starboard)
        R.col(2) << 0.0, 1.0, 0.0; // body +z points to inertial +y 

        Eigen::Vector4d baseOrientation = QuaternionTools::RotationMatrixToQuat(R);

        double rollRad = initRoll * M_PI / 180;
        double pitchRad = initPitch * M_PI / 180;
        double yawRad = initYaw * M_PI / 180;

        // Creating rotation quaternions for each body axis
        Eigen::Vector3d bodyX(1.0, 0.0, 0.0);
        Eigen::Vector3d bodyY(0.0, 1.0, 0.0);
        Eigen::Vector3d bodyZ(0.0, 0.0, 1.0);
        Eigen::Vector4d rollQuat = QuaternionTools::AxisAngleToQuat(bodyX, rollRad);
        Eigen::Vector4d pitchQuat = QuaternionTools::AxisAngleToQuat(bodyY, pitchRad);
        Eigen::Vector4d yawQuat = QuaternionTools::AxisAngleToQuat(bodyZ, yawRad);

        initialOrientation = QuaternionTools::multiply(baseOrientation,
                            QuaternionTools::multiply(QuaternionTools::multiply(yawQuat, pitchQuat), rollQuat));
    }
};
