/*
~ World.cpp ~

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

    World(const string& worldName, double r, double m);

    // returns the magnitude of gravitational acceleration for a given altitude
    double getGravitationalAccel(const double altitude) const;

    // predefined worlds
    static World Earth();
    
    static World Moon();
    
    static World Mars();
};