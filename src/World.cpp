/*
~ World.h ~

Owns:
- Parameters for the planet/environment
*/

#pragma once

#include <Eigen/Dense>
#include <string>
#include <cmath>
using namespace std;

class World {
public:
    string name;
    double radius; // m (though the world is flat, radius will be utilized for gravity calculations)
    double mass; // kg
    const double G = 6.6743e-11; // m^3*kg^-1*s^-2

    World(const string& worldName, double r, double m) {
        name = worldName;
        radius = r;
        mass = m;
    }

    // returns the magnitude of gravitational acceleration for a given altitude
    double getGravitationalAccel(const double altitude) const{
        double r = radius + altitude;
        return (G * mass)/(r*r); // m/s^2 (note that this is the MAGNITUDE)
    }

    // predefined worlds
    static World Earth() {
        return World("Earth", 6.371e6, 5.972e24);
    }
    
    static World Moon() {
        return World("the Moon", 1.737e6, 7.347e22);
    }
    
    static World Mars() {
        return World("Mars", 3.389e6, 6.417e23);
    }
};