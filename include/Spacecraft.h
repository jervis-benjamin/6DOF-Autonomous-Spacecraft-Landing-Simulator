/*
~ Spacecraft.cpp ~

Owns:
- Initial state vector
- Mass, CG, inertia values
- CG, Inertia update function
- Vehicle geometry dimensions
*/

#pragma once

#include <Eigen/Dense>
#include <cmath>

#include "QuaternionTools.h"

using namespace std;

class Spacecraft {
public:

    Spacecraft();

    // mass
    const double dryMass = 7000.0; // kg (roughly the mass dry mass + ascent stage prop mass of lunar module)
    double propellantMass = 8200.0; // kg
    double initialPropellantMass = propellantMass; // kg
    double totalMass = dryMass + propellantMass; // kg

    // geometry
    const double vehWidth = 4.2; // m (width of vehicle, roughly the width of the lunar module with its legs undeployed)
    const double vehHeight = 7.0; // m (height of vehicle, roughly the height of the lunar module)
    const double tankWidth = 1.68; // m (width of tank, size based on a mixture density of 1150 kg/m^3 given a tank height of below)
    const double tankHeight = 3.2; // m (height of tank, roughly the height of the descent stage)

    // mass-geometry
    Eigen::Vector3d cg = Eigen::Vector3d::Zero(); // m (distance to cg from bottom of the vehicle in the body frame)
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero(); // kg*m^2 (about the center of mass)

    // other specs
    double touchdownVelocityLimit = 1.5; // m/s (magnitude of the max touchdown velocity tolerance of the lander)

    // default initial conditions
    Eigen::Vector3d initialPosition; // m
    Eigen::Vector3d initialVelocity; // m/s
    Eigen::Vector4d initialOrientation; // w, x, y, z (quaternion)
    Eigen::Vector3d initialAngularVelocity; // rad/s

    /*
    Center of gravity and moment of inertia equations were derived by simplfying the vehicle as two cylinders.
    One with the dimensions of the vehicle and with the mass of dryMass, and the other with the dimensions 
    of the propellant column in the tank with the mass of propellantMass. The propellant column/tank is 
    positioned such that it is in the center of the vehicle with the same height of the descent stage.
    */
    void updateCG();

    void updateInertia();

    void updateMassProperties();
};