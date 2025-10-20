#pragma once

#include <Eigen/Dense>
#include <string>
#include <iostream>
using namespace std;

class Celestial {
public:
    static constexpr double G = 6.67430e-11;  // gravitational constant (m^3/kg/s^2)
    
    string name;
    double radius; // m
    double mass; // kg
    
    Celestial(const string& bodyName, double r, double m) 
        : name(bodyName), radius(r), mass(m) {}
    
    
    Eigen::Vector3d getGravityVector(const Eigen::Vector3d& position) const {
        double r = position.norm();
        return -G * mass / (r*r*r) * position;
    }
    
    
    double getAltitude(const Eigen::Vector3d& position) const {
        return position.norm() - radius;
    }
    
    
    double getOrbitalVelocity(double r) const {
        return sqrt(G * mass / r);
    }

    // predefined objects
    static Celestial Earth() {
        return Celestial("Earth", 6371000.0, 5.972e24);
    }
    
    static Celestial Moon() {
        return Celestial("Moon", 1737400.0, 7.342e22);
    }
    
    static Celestial Mars() {
        return Celestial("Mars", 3389500.0, 6.417e23);
    }
    
};
