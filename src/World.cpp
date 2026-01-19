/*
~ World.cpp ~

Owns:
- Parameters for the planet/environment
*/

#include <Eigen/Dense>
#include <string>
#include <cmath>

#include "../include/World.h"

using namespace std;

World::World(const string& worldName, double r, double m) {
    name = worldName;
    radius = r;
    mass = m;
}

// returns the magnitude of gravitational acceleration for a given altitude
double World::getGravitationalAccel(const double altitude) const{
    double r = radius + altitude;
    return (G * mass)/(r*r); // m/s^2 (note that this is the MAGNITUDE)
}

// predefined worlds
World World::Earth() {
    return World("Earth", 6.371e6, 5.972e24);
}

World World::Moon() {
    return World("the Moon", 1.737e6, 7.347e22);
}

World World::Mars() {
    return World("Mars", 3.389e6, 6.417e23);
}