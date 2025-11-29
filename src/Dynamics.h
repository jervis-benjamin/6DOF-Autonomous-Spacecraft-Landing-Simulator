/*
~ Dynamics.h ~

Owns: 
- State vector
- State vector updates
*/


#pragma once

#include <Eigen/Dense>
#include "Spacecraft.h"
#include "QuaternionTools.h"
#include "World.h"
using namespace std;

class Dynamics {

private:
    const World& world;
    const Spacecraft& spacecraft;

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

        double currentMass = spacecraft.totalMass;
        Eigen::Vector3d weight(0.0, 0.0, -world.getGravitationalAccel(pos.z()) * currentMass);
        Eigen::Vector4d worldQuat = QuaternionTools::toWorld(quat);
        Eigen::Vector3d forceInertial = QuaternionTools::rotateVector(worldQuat, bodyForce);
        Eigen::Vector3d totalForce = forceInertial + weight;
        deriv.velocityDot = totalForce / currentMass;

        deriv.orientationDot = QuaternionTools::derivative(quat, angVel);

        Eigen::Matrix3d currentInertia = spacecraft.inertia;
        Eigen::Vector3d Iw = currentInertia * angVel;
        Eigen::Vector3d omegaCrossIw = angVel.cross(Iw);
        Eigen::Vector3d IwDot = bodyTorque - omegaCrossIw;
        deriv.angularVelocityDot = currentInertia.inverse() * IwDot;

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
        // TODO: accommodate for landing gear height in offset (add landing gear height in spacecraft.h)
        
        Eigen::Vector3d bottomDist_body(-spacecraft.cg.x(), 0.0, 0.0); // distance to the bottom of the vehicle in body frame

        // rotate into the inertial frame
        Eigen::Vector4d worldQuat = QuaternionTools::toWorld(orientation);
        Eigen::Vector3d bottomDist_world = QuaternionTools::rotateVector(worldQuat, bottomDist_body);

        double bottomAltitude = position.z() + bottomDist_world.z(); // z position in the world frame of the bottom of the vehicle

        if (bottomAltitude <= 0.0) { // handle clipping through the floor
        
            position.z() -= bottomAltitude;
            impactVelocity = velocity.norm();
            
            // using the definition of the dot product to find the tilt angle (angle between the vehicle and the vertical)
            Eigen::Vector3d bodyUp(1.0, 0.0, 0.0); 
            Eigen::Vector3d rocketUpInertial = QuaternionTools::rotateVector(worldQuat, bodyUp);
            Eigen::Vector3d inertialUp(0.0, 0.0, 1.0);
            double cosAngle = rocketUpInertial.dot(inertialUp);
            cosAngle = std::clamp(cosAngle, -1.0, 1.0); // clamping to [-1, 1] to avoid numerical errors in acos
            double tiltAngle = acos(cosAngle) * (180.0 / PI);
            
            if (tiltAngle < 5.0) { // threshold for a "safe" landing (minimal chance to tip over)
                velocity.setZero();
                angularVelocity.setZero();
                landed = true;
            } else { 
                velocity.setZero();
                angularVelocity.setZero();
                landed = true;
                tippedOver = true; // vehicle is prone to or has tipped over
            }
        }
    }


public:
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector4d orientation;
    Eigen::Vector3d angularVelocity;
    bool landed;
    bool tippedOver; // if the landing was at too much of an angle
    // TODO: turn tippedOver to false if descent rate exceeds threshold set by the final guidance phase.
    double impactVelocity;

    Dynamics(const Spacecraft& sc, const World& w) : spacecraft(sc), world(w){
        position = sc.initialPosition;
        velocity = sc.initialVelocity;
        orientation = QuaternionTools::toRelative(sc.initialOrientation);
        angularVelocity = sc.initialAngularVelocity;
        landed = false;
        tippedOver = false;
        impactVelocity = 999999.9; // set to NaN
    }
    
    Eigen::Vector4d getWorldOrientation() const {
        return QuaternionTools::toWorld(orientation);
    }

    void update(double dt, const Eigen::Vector3d& bodyForce, const Eigen::Vector3d& bodyTorque) {
        integrateRK4(dt, bodyForce, bodyTorque);
        handleCollisions();
    }

};
