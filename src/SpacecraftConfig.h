#pragma once

#include <Eigen/Dense>
using namespace std;

struct SpacecraftConfig {
    double mass;                // kg
    Eigen::Matrix3d inertiaTensor; // kg*m^2
    double gravity;             // m/s^2
    Eigen::Vector3d initialPosition;      // m
    Eigen::Vector3d initialVelocity;      // m/s
    Eigen::Vector4d initialOrientation;   // quaternion (w, x, y, z)
    Eigen::Vector3d initialAngularVelocity; // rad/s

    SpacecraftConfig() {
        mass = 1000.0;
        inertiaTensor = Eigen::Matrix3d::Zero();
        inertiaTensor(0,0) = 500.0;
        inertiaTensor(1,1) = 700.0;
        inertiaTensor(2,2) = 300.0;
        gravity = 1.62; // m/s^2
        initialPosition << 50, 0.0, 100;
        initialVelocity << 1200, 30, 0.0;
        initialOrientation << 1.0, 0.0, 0.0, 0.0;
        initialAngularVelocity << 0.0, 0.0, 0.0;
    }
};
