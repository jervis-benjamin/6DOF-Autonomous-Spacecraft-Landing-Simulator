/*
~ Guidance.cpp ~

Owns:
- Scheduling velocity setpoints during various phases of descent
- Scheduling roll setpoint
*/

#include <Eigen/Dense>

#include "../include/Guidance.h"
#include "../include/Dynamics.h"
#include "../include/Spacecraft.h"
#include "../include/QuaternionTools.h"
#include "../include/World.h"

using namespace std;

Guidance::Guidance(Spacecraft& sc, const Dynamics& dn, const World& w) : spacecraft(sc), dynamics(dn), world(w){}

double Guidance::X_correctiveVelocity(double dt, double position, double target, double currVelocity){
    // returns a command velocity to correct current position based on a simple function
    double error = target - position;
    xError += error * dt;
    xError = clamp(xError, -30.0, 30.0);

    double cmdVelocity = (xKp * error) + (xKi * xError) - (xKd * currVelocity); 

    return cmdVelocity;
}

double Guidance::Y_correctiveVelocity(double dt, double position, double target, double currVelocity){
    // returns a command velocity to correct current position based on a simple function
    double error = target - position;
    yError += error * dt;
    yError = clamp(yError, -30.0, 30.0);

    double cmdVelocity = (yKp * error) + (yKi * yError) - (yKd * currVelocity); 

    return cmdVelocity;
}

double Guidance::getHorizontalDistance(){
    // horizontal distance from current position to target 
    double xDist = dynamics.position.x() - landingTarget.x();
    double yDist = dynamics.position.y() - landingTarget.y();
    return sqrt(pow(xDist, 2.0) + pow(yDist, 2.0));
}


void Guidance::update(double dt){
    // reset all guidance values to default
    velocitySetpoints = dynamics.velocity;
    ignoreRoll = true;
    maintainOrientation = false;
    guidanceState = 0;

    // gather data needed to trigger guidance flags
    double currentRange = dynamics.position.x();
    double currentCrossRange = dynamics.position.y();
    double currentAlt = dynamics.position.z();
    double distToTarget = getHorizontalDistance();


    if (currentAlt >= approachPhaseAlt){ // BREAKING PHASE
        guidanceState = 1.0;

        if (dynamics.velocity.z() > breakingDescentRate){
            // to prevent vehicle flipping over when it starts at a velocity greater than breakingDescentRate
            velocitySetpoints.z() = breakingDescentRate;
        }

        velocitySetpoints.y() = Y_correctiveVelocity(dt, currentCrossRange, landingTarget.y(), dynamics.velocity.y());

        if (distToTarget >= stopBreakingBurn){
            velocitySetpoints.x() = breakingVelocity;
        } else{
            guidanceState = 1.5;
            velocitySetpoints.x() = X_correctiveVelocity(dt, currentRange, landingTarget.x(), dynamics.velocity.x());
        }

    } else if (currentAlt >= beginTerminalPhaseAlt){ // APPROACH PHASE
        guidanceState = 2.0;
        
        ignoreRoll = false;

        velocitySetpoints.z() = approachDescentRate;

        velocitySetpoints.y() = Y_correctiveVelocity(dt, currentCrossRange, landingTarget.y(), dynamics.velocity.y());

        if (distToTarget >= stopBreakingBurn){
            velocitySetpoints.x() = approachVelocity;
        } else{
            guidanceState = 2.5;
            velocitySetpoints.x() = X_correctiveVelocity(dt, currentRange, landingTarget.x(), dynamics.velocity.x());
        }

    } else if (currentAlt >= finalPhaseAlt){ // BEGINNING OF TERMINAL DESCENT
        guidanceState = 3.0;
        ignoreRoll = false;
        velocitySetpoints.x() = X_correctiveVelocity(dt, currentRange, landingTarget.x(), dynamics.velocity.x());
        //velocitySetpoints.x() = 0.0;
        //velocitySetpoints.y() = 0.0;
        velocitySetpoints.y() = Y_correctiveVelocity(dt, currentCrossRange, landingTarget.y(), dynamics.velocity.y());
        velocitySetpoints.z() = terminalDescentRate1;

    } else { // FINAL DESCENT PHASE
        guidanceState = 4.0;
        ignoreRoll = false;
        velocitySetpoints.x() = X_correctiveVelocity(dt, currentRange, landingTarget.x(), dynamics.velocity.x());
        //velocitySetpoints.x() = 0.0;
        //velocitySetpoints.y() = 0.0;
        velocitySetpoints.y() = Y_correctiveVelocity(dt, currentCrossRange, landingTarget.y(), dynamics.velocity.y());
        velocitySetpoints.z() = terminalDescentRate2;
        //maintainOrientation = true;

    }
}