#pragma once

#include <Eigen/Dense>
#include "SpacecraftConfig.h"
#include "QuaternionTools.h"
using namespace std;

class Dynamics {
public:
    SpacecraftConfig config;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector4d orientation;
    Eigen::Vector3d angularVelocity;

    Dynamics(const SpacecraftConfig& cfg) {
        config = cfg;
        position = cfg.initialPosition;
        velocity = cfg.initialVelocity;
        orientation = cfg.initialOrientation;
        angularVelocity = cfg.initialAngularVelocity;
    }

    void update(double dt, const Eigen::Vector3d& bodyForce, const Eigen::Vector3d& bodyTorque) {
        Eigen::Vector3d gravity(0.0, 0.0, -config.gravity * config.mass);

        Eigen::Vector3d forceInertial = QuaternionTools::rotateVector(orientation, bodyForce);

        Eigen::Vector3d totalForce = forceInertial + gravity;
        Eigen::Vector3d acceleration = totalForce / config.mass;

        velocity += acceleration * dt;
        position += velocity * dt;

        if (position.z() < 0.0) {
            position.z() = 0; //preventing spacecraft from clipping the floor
            velocity.z() = 0;
            velocity.x() /= 2; // TODO: implement more realistic lateral velocity reduction
            velocity.y() /= 2;
            if (velocity.x() < 1e-5){
                velocity.x() = 0;
            }
            if (velocity.y() < 1e-5){
                velocity.y() = 0;
            }
        }
        

        Eigen::Vector3d Iw = config.inertiaTensor * angularVelocity;
        Eigen::Vector3d omegaCrossIw = angularVelocity.cross(Iw);
        Eigen::Vector3d angularAccel = bodyTorque - omegaCrossIw;
        angularAccel(0) /= config.inertiaTensor(0,0);
        angularAccel(1) /= config.inertiaTensor(1,1);
        angularAccel(2) /= config.inertiaTensor(2,2);

        angularVelocity += angularAccel * dt;

        Eigen::Vector4d qDot = QuaternionTools::derivative(orientation, angularVelocity);

        orientation += qDot * dt;

        QuaternionTools::normalize(orientation);
    }
};
