/*
~ Spacecraft.cpp ~

Owns:
- Initial state vector
- Mass, CG, inertia values
- CG, Inertia update function
- Vehicle geometry dimensions
*/

#include <Eigen/Dense>
#include <cmath>

#include "../include/Spacecraft.h"
#include "../include/QuaternionTools.h"

using namespace std;

Spacecraft::Spacecraft() {

    initialPropellantMass = propellantMass;
    updateMassProperties();

    // realistic apollo numbers
    //initialPosition << 350000, -500, 15000; // 350km away from target with a 0.5km skew, comming in at 15km of alt (scaled roughly Apollo landing trajectory in a round world)
    //initialVelocity << -1700, -5, 0; // approximately 1700 m/s horizontal velocity (with a small lateral drift) and close to 0 horizontal velocity (rougly conditions close to the begining of powered decent during Apollo)
    
    // starting values adjusted for flat moon with no orbital velocity, relative to target point
    initialPosition << 35000, 50, 15000; // 35km away from target with a 50 m skew, comming in at 15km of alt (scaled roughly Apollo landing trajectory in a flat world)
    initialVelocity << -400, 5, -15; // post deorbit durn velocity roughly scaled for flat world physics

    // initialPosition << 0.0, 0.0, 1000.0; // for testing
    // initialVelocity << 0.0, 0.0, 0.0; // for testing

    initialAngularVelocity << 0.0, 0.0, 0.0; 
    
    Eigen::Vector4d referenceOrientation(1.0, 0.0, 0.0, 0.0);
    referenceOrientation = QuaternionTools::rotateQuat(referenceOrientation, 'y', -90.0); // orienting the body axis such that body +X is with the inertial +Z
    QuaternionTools::setReference(referenceOrientation);

    Eigen::Vector4d relativeOrientation(1.0, 0.0, 0.0, 0.0); // Start upright in the "relative" frame
    initialOrientation = QuaternionTools::toWorld(relativeOrientation);
    initialOrientation = QuaternionTools::rotateQuat(initialOrientation, 'y', 90.0); // setting initial orientation
}

/*
Center of gravity and moment of inertia equations were derived by simplfying the vehicle as two cylinders.
One with the dimensions of the vehicle and with the mass of dryMass, and the other with the dimensions 
of the propellant column in the tank with the mass of propellantMass. The propellant column/tank is 
positioned such that it is in the center of the vehicle with the same height of the descent stage.
*/
void Spacecraft::updateCG(){
    double propHeight = (propellantMass * tankHeight) / initialPropellantMass;
    cg(0,0) = (dryMass * vehHeight + propellantMass * propHeight) / (2.0 * totalMass);
}

void Spacecraft::updateInertia() {
    double propHeight = (propellantMass * tankHeight) / initialPropellantMass;
    inertia(0,0) = (1.0 / 8.0) * (dryMass * pow(vehWidth, 2.0) + propellantMass * pow(tankWidth, 2.0));
    inertia(1,1) =  (1.0 / 48.0) * ( 
                                        dryMass        * (3.0 * pow(vehWidth, 2.0)  + 16.0 * pow(vehHeight, 2.0))
                                        + propellantMass * (3.0 * pow(tankWidth, 2.0) + 16.0 * pow(propHeight, 2.0)) 
                                    )
                                    - pow( (dryMass * vehHeight + propellantMass * propHeight), 2.0 ) / (4.0 * totalMass);
    inertia(2,2) = inertia(1,1);
}

void Spacecraft::updateMassProperties(){
    totalMass = dryMass + propellantMass;
    updateCG();
    updateInertia();
}