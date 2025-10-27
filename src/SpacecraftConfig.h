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
        initialPosition << 0.0, 0.0, 0.0; // for testing
        initialVelocity << 0.0, 0.0, 0.0; // for testing
        initialAngularVelocity << 0.0, 0.0, 0.0;

        QuaternionTools::setOffsets(-90.0, 0.0, 0.0); // (p, y, r) defining the zero reference orientation for the body from the world frame
        initialOrientation << 1.0, 0.0, 0.0, 0.0; // initially alligned with the inertial frame
        initialOrientation = QuaternionTools::rotateQuat(initialOrientation, 'y', -180.0); // gives us a retrograde orientation at the start of powered descent
    }
};
