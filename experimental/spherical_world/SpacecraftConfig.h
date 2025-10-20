#pragma once

#include <Eigen/Dense>
#include <cmath>
#include "Celestial.h"
using namespace std;

struct SpacecraftConfig {
    double mass;                // kg
    Eigen::Matrix3d inertiaTensor; // kg*m^2

    // initial states
    Eigen::Vector3d initialPosition;      // m at body center, in ECI frame
    Eigen::Vector3d initialVelocity;      // m/s in ECI frame
    Eigen::Vector4d initialOrientation;   // quaternion 
    Eigen::Vector3d initialAngularVelocity; // rad/s

    SpacecraftConfig(const Celestial& body, 
                     double startAltitude = 50000.0,      // m above surface
                     double startLatitude = 0.0,          // degrees (-90 to +90, 0=equator)
                     double startLongitude = -10.0,       // degrees (0=prime meridian)
                     double approachAzimuth = 90.0,       // degrees (0=north, 90=east, 180=south, 270=west)
                     double orbitalFraction = 0.95,       // fraction of circular orbital velocity (< 1.0 = suborbital)
                     double descentRate = 100.0) {        // m/s (positive = descending)


        mass = 1000.0;
        inertiaTensor = Eigen::Matrix3d::Zero();
        inertiaTensor(0,0) = 500.0;
        inertiaTensor(1,1) = 700.0;
        inertiaTensor(2,2) = 300.0;
            
        double lat_rad = startLatitude * M_PI / 180.0;
        double lon_rad = startLongitude * M_PI / 180.0;
        double azimuth_rad = approachAzimuth * M_PI / 180.0;

        // position of spacecraft in ECI frame (spherical coordinates to cartesian)
        double r = body.radius + startAltitude;
        initialPosition << r * cos(lat_rad) * cos(lon_rad), r * cos(lat_rad) * sin(lon_rad), r * sin(lat_rad);

        // building a local east north up (ENU) frame to describe initial velocity conditions
        Eigen::Vector3d radial = initialPosition.normalized();
        Eigen::Vector3d northPoleAxis(0.0, 0.0, 1.0);  // Z-axis points to north pole
        Eigen::Vector3d east = northPoleAxis.cross(radial);

        if (east.norm() < 1e-6) { // at the poles, east may yield 
            east << 1.0, 0.0, 0.0;
        }
        east.normalize();

        Eigen::Vector3d north = radial.cross(east);
        north.normalize();

        double v_circular = body.getOrbitalVelocity(r);
        double v_tangential = v_circular * orbitalFraction;
        double v_descent = -descentRate;
        Eigen::Vector3d horizontalDirection = cos(azimuth_rad) * north + sin(azimuth_rad) * east;
        initialVelocity = v_tangential * horizontalDirection + v_descent * radial; 
        
        initialOrientation << 1.0, 0.0, 0.0, 0.0;
        initialAngularVelocity << 0.0, 0.0, 0.0;
    }
    
    SpacecraftConfig() : SpacecraftConfig(Celestial::Earth()) {} // set body of focus

};