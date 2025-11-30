/*
~ Spacecraft.h ~

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
    // mass
    const double dryMass = 7000.0; // kg (roughly the mass dry mass + ascent stage prop mass of lunar module)
    double propellantMass; // kg
    double initialPropellantMass; // kg
    double totalMass; // kg

    // geometry
    const double vehWidth = 4.2; // m (width of vehicle, roughly the width of the lunar module with its legs undeployed)
    const double vehHeight = 7.0; // m (height of vehicle, roughly the height of the lunar module)
    const double tankWidth = 1.68; // m (width of tank, size based on a mixture density of 1150 kg/m^3 given a tank height of below)
    const double tankHeight = 3.2; // m (height of tank, roughly the height of the descent stage)

    // mass-geometry
    Eigen::Vector3d cg; // m (distance to cg from bottom of the vehicle in the body frame)
    Eigen::Matrix3d inertia; // kg*m^2 (about the center of mass)

    // other specs
    double targetDescentRate = 0; // m/s target descent rate for this spacecraft (put in guidance once it has been implemented)

    // initial conditions
    Eigen::Vector3d initialPosition; // m
    Eigen::Vector3d initialVelocity; // m/s
    Eigen::Vector4d initialOrientation; // w, x, y, z
    Eigen::Vector3d initialAngularVelocity; // rad/s

    
    Spacecraft() {
        propellantMass = 8200 / 3; // kg (roughly the descent stage prop mass of the lunar module)
        initialPropellantMass = 8200;
        //initialPropellantMass = propellantMass; 
        totalMass = dryMass + propellantMass;
    
        cg = Eigen::Vector3d::Zero();
        updateCG();
        
        inertia = Eigen::Matrix3d::Zero();
        updateInertia();
        
        // realistic apollo/round-moon numbers
        //initialPosition << 350000, -2000, 15000; // 350km away from target with a 2km skew, comming in at 15km of alt (scaled roughly Apollo landing trajectory in a flat world)
        //initialVelocity << -1700, -5, 0; // approximately 1700 m/s horizontal velocity (with a small lateral drift) and close to 0 horizontal velocity (rougly conditions close to the begining of powered decent during Apollo)
        
        // starting values adjusted for flat moon
        //initialPosition << 200000, -2000, 15000; // 350km away from target with a 2km skew, comming in at 15km of alt (scaled roughly Apollo landing trajectory in a flat world)
        //initialVelocity << -1700, -5, 0;

        initialPosition << 0.0, 0.0, 3000.0; // for testing
        initialVelocity << 0.0, 0.0, 0.0; // for testing

        initialAngularVelocity << 0.0, 0.0, 0.0; 
        
        Eigen::Vector4d referenceOrientation(1.0, 0.0, 0.0, 0.0);
        referenceOrientation = QuaternionTools::rotateQuat(referenceOrientation, 'y', -90.0); // orienting the body axis such that body +X is with the inertial +Z
        QuaternionTools::setReference(referenceOrientation);

        Eigen::Vector4d relativeOrientation(1.0, 0.0, 0.0, 0.0);
        initialOrientation = QuaternionTools::toWorld(relativeOrientation);
        initialOrientation = QuaternionTools::rotateQuat(initialOrientation, 'y', 0.0); // setting initial orientation

    }

    /*
    Center of gravity and moment of inertia equations were derived by simplfying the vehicle as two cylinders.
    One with the dimensions of the vehicle and with the mass of dryMass, and the other with the dimensions 
    of the propellant column in the tank with the mass of propellantMass. The propellant column / tank is 
    positioned such that it is in the center of the vehicle with the same height of the descent stage.
    */
    void updateCG(){
        double propHeight = (propellantMass * tankHeight) / initialPropellantMass;
        cg(0,0) = (dryMass * vehHeight + propellantMass * propHeight) / (2.0 * totalMass);
    }

    void updateInertia() {
        double propHeight = (propellantMass * tankHeight) / initialPropellantMass;
        inertia(0,0) = (1.0 / 8.0) * (dryMass * pow(vehWidth, 2.0) + propellantMass * pow(tankWidth, 2.0));
        inertia(1,1) =  (1.0 / 48.0) * ( 
                                           dryMass        * (3.0 * pow(vehWidth, 2.0)  + 16.0 * pow(vehHeight, 2.0))
                                         + propellantMass * (3.0 * pow(tankWidth, 2.0) + 16.0 * pow(propHeight, 2.0)) 
                                        )
                                     - pow( (dryMass * vehHeight + propellantMass * propHeight), 2.0 ) / (4.0 * totalMass);
        inertia(2,2) = inertia(1,1);
    }
};