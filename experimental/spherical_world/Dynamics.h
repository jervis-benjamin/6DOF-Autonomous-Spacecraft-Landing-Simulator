#pragma once

#include <Eigen/Dense>
#include "SpacecraftConfig.h"
#include "QuaternionTools.h"
#include "Celestial.h"
using namespace std;

class Dynamics {

private:

    Celestial body;

    struct StateDerivative {
        Eigen::Vector3d positionDot;
        Eigen::Vector3d velocityDot;
        Eigen::Vector4d orientationDot;
        Eigen::Vector3d angularVelocityDot;

        StateDerivative operator+(const StateDerivative& other) const {
            StateDerivative result;
            result.positionDot = positionDot + other.positionDot;
            result.velocityDot = velocityDot + other.velocityDot;
            result.orientationDot = orientationDot + other.orientationDot;
            result.angularVelocityDot = angularVelocityDot + other.angularVelocityDot;
            return result;
        }

        StateDerivative operator*(double scalar) const {
            StateDerivative result;
            result.positionDot = positionDot * scalar;
            result.velocityDot = velocityDot * scalar;
            result.orientationDot = orientationDot * scalar;
            result.angularVelocityDot = angularVelocityDot * scalar;
            return result;
        }
    };

    StateDerivative computeDerivatives(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const Eigen::Vector4d& quat, const Eigen::Vector3d& angVel, const Eigen::Vector3d& bodyForce, const Eigen::Vector3d& bodyTorque) const {
        
        StateDerivative deriv;

        deriv.positionDot = vel;

        Eigen::Vector3d gravitationalAccel = body.getGravityVector(pos);
        Eigen::Vector3d forceInertial = QuaternionTools::rotateVector(quat, bodyForce);
        Eigen::Vector3d totalAccel = (forceInertial/config.mass) + gravitationalAccel;
        deriv.velocityDot = totalAccel;

        deriv.orientationDot = QuaternionTools::derivative(quat, angVel);

        Eigen::Vector3d Iw = config.inertiaTensor * angVel;
        Eigen::Vector3d omegaCrossIw = angVel.cross(Iw);
        Eigen::Vector3d IwDot = bodyTorque - omegaCrossIw;
        deriv.angularVelocityDot = config.inertiaTensor.inverse() * IwDot;

        return deriv;
    }

    void integrateRK4(double dt, const Eigen::Vector3d& bodyForce, const Eigen::Vector3d& bodyTorque) {
        StateDerivative k1 = computeDerivatives(
            position, velocity, orientation, angularVelocity, 
            bodyForce, bodyTorque);

        Eigen::Vector3d pos2 = position + k1.positionDot * (0.5 * dt);
        Eigen::Vector3d vel2 = velocity + k1.velocityDot * (0.5 * dt);
        Eigen::Vector4d quat2 = orientation + k1.orientationDot * (0.5 * dt);
        QuaternionTools::normalize(quat2);
        Eigen::Vector3d angVel2 = angularVelocity + k1.angularVelocityDot * (0.5 * dt);
        
        StateDerivative k2 = computeDerivatives(
            pos2, vel2, quat2, angVel2, 
            bodyForce, bodyTorque);

        Eigen::Vector3d pos3 = position + k2.positionDot * (0.5 * dt);
        Eigen::Vector3d vel3 = velocity + k2.velocityDot * (0.5 * dt);
        Eigen::Vector4d quat3 = orientation + k2.orientationDot * (0.5 * dt);
        QuaternionTools::normalize(quat3);
        Eigen::Vector3d angVel3 = angularVelocity + k2.angularVelocityDot * (0.5 * dt);
        
        StateDerivative k3 = computeDerivatives(
            pos3, vel3, quat3, angVel3, 
            bodyForce, bodyTorque);

        Eigen::Vector3d pos4 = position + k3.positionDot * dt;
        Eigen::Vector3d vel4 = velocity + k3.velocityDot * dt;
        Eigen::Vector4d quat4 = orientation + k3.orientationDot * dt;
        QuaternionTools::normalize(quat4);
        Eigen::Vector3d angVel4 = angularVelocity + k3.angularVelocityDot * dt;
        
        StateDerivative k4 = computeDerivatives(
            pos4, vel4, quat4, angVel4, 
            bodyForce, bodyTorque);

        position += (k1.positionDot + k2.positionDot * 2.0 + k3.positionDot * 2.0 + k4.positionDot) * (dt / 6.0);
        velocity += (k1.velocityDot + k2.velocityDot * 2.0 + k3.velocityDot * 2.0 + k4.velocityDot) * (dt / 6.0);
        orientation += (k1.orientationDot + k2.orientationDot * 2.0 + k3.orientationDot * 2.0 + k4.orientationDot) * (dt / 6.0);
        angularVelocity += (k1.angularVelocityDot + k2.angularVelocityDot * 2.0 + k3.angularVelocityDot * 2.0 + k4.angularVelocityDot) * (dt / 6.0);

        QuaternionTools::normalize(orientation);
    }

    void handleCollisions() { 
        double altitude = body.getAltitude(position);
    
        if (altitude <= 0.0) {
            position = position.normalized() * body.radius; // bringing spacecraft back to the surface of the planet
            impactVelocity = velocity;
            velocity.setZero();
            angularVelocity.setZero();
            
            landed = true;
        }
    }

public:
    SpacecraftConfig config;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d impactVelocity;
    Eigen::Vector4d orientation;
    Eigen::Vector3d angularVelocity;
    bool landed;

    Dynamics(const SpacecraftConfig& cfg, const Celestial& celestialBody) : body(celestialBody) {
        config = cfg;
        position = cfg.initialPosition;
        velocity = cfg.initialVelocity;
        orientation = cfg.initialOrientation;
        angularVelocity = cfg.initialAngularVelocity;
        landed = false;
    }

    void update(double dt, const Eigen::Vector3d& bodyForce, const Eigen::Vector3d& bodyTorque) {
        if (!landed){
            integrateRK4(dt, bodyForce, bodyTorque);
            handleCollisions();
        }
    }

    // get functions for printouts
    double getAltitude() const {
        return body.getAltitude(position);
    }
    
    double getSpeed() const {
        return velocity.norm();
    }

    double getImpactSpeed() const {
        return impactVelocity.norm();
    }
    
    const Celestial& getCelestialBody() const {
        return body;
    }
};