/*
~ Guidance.h ~

Owns:
- Scheduling velocity setpoints during various phases of descent
- Scheduling roll setpoint
*/

/*
as of now, the guidance class phase triggers are too dependant on the spacecraft's initial states. 
later on, it would be key to implement guidance that works for a large range of initial states and is more robust
*/
#pragma once

#include <Eigen/Dense>
#include "Dynamics.cpp"
#include "Spacecraft.cpp"
#include "QuaternionTools.cpp"
#include "World.cpp"
using namespace std;

class Guidance{

private:
    const Spacecraft& spacecraft;
    const Dynamics& dynamics;
    const World& world;

    /*
    landing target is at (0, 0, 0), and spacecraft position is expressed as the relative distance to target.
    that being said, in the future maybe we could add a functionality to land it any coordinate.
    */
    Eigen::Vector3d landingTarget{0.0, 0.0, 0.0};

    // guidance key triggers (find better variable names)
    /* RANGES */
    double stopBreakingBurn = 500; // m end breaking phase once we are this close to the target      
    /* ALTITUDES */
    double approachPhaseAlt = 10000; // m (altitude at which the spacecraft decends at secondDecentRate velocity)
    double lockVeclocityAlt = 3000; // m (altitude at which x and y velocity will be commanded to zero)
    double lockOrientationAlt = 1000; // m (altitude at which vehicle will be oriented level with the ground)

    // horizontal breaking velocity
    double breakingVelocity = -94.3; // m/s
    double approachVelocity = -74.3; // m/s

    // descent rates
    double breakingDescentRate = -60.0; // m/s
    double approachDescentRate = -30.0; // m/s
    double lockVeclocityDescentRate = -15.0; // m/s
    double finalDescentRate = -1.4; // m/s

    // KD gains for corrective velocity controller
    double Kp = 0.2;
    double Kd = 1.8;

    double correctiveVelocity(double position, double target, double currVelocity){
        // returns a command velocity to correct current position based on a simple function
        double error = target - position;

        double cmdVelocity = Kp * error - Kd * currVelocity; 

        return cmdVelocity;
    }

    // double impactDistanceToTarget(Eigen::Vector3d velocity, Eigen::Vector3d position, double gravityMag){
    //     // returns the distance from the IIP to the landing target

    //     // determine IIP x-cor
    //     double IIP_x = position.x() + (velocity.x() / gravityMag) * (velocity.z() + sqrt(pow(velocity.z(), 2.0) + 2*position.z()*gravityMag));

    //     // determine IIP y-cor
    //     double IIP_y = position.y() + (velocity.y() / gravityMag) * (velocity.z() + sqrt(pow(velocity.z(), 2.0) + 2*position.z()*gravityMag));

    //     // determine distance from target
    //     double distanceToTarget = sqrt( pow((IIP_x - landingTarget.x()), 2.0) + pow((IIP_y - landingTarget.y()), 2.0) );
    //     return distanceToTarget;
    // }

    double getHorizontalDistance(){
        // horizontal distance from current position to target 
        double xDist = dynamics.position.x() - landingTarget.x();
        double yDist = dynamics.position.y() - landingTarget.y();
        return sqrt(pow(xDist, 2.0) + pow(yDist, 2.0));
    }

public:
    Eigen::Vector3d velocitySetpoints{0.0, 0.0, 0.0};
    bool ignoreRoll = true;
    double rollSetpointDeg = 0.0;
    bool maintainOrientation = false;
    Eigen::Vector3d eulerSetpointsDeg{0.0, 0.0, 0.0}; // final landing orientation
    
    Guidance(Spacecraft& sc, const Dynamics& dn, const World& w) : spacecraft(sc), dynamics(dn), world(w){}

    void update(){
        // reset all guidance values to default
        velocitySetpoints = dynamics.velocity;
        ignoreRoll = true;
        maintainOrientation = false;

        // gather data needed to trigger guidance flags
        double currentRange = dynamics.position.x();
        double currentCrossRange = dynamics.position.y();
        double currentAlt = dynamics.position.z();
        double distToTarget = getHorizontalDistance();


        if (currentAlt >= approachPhaseAlt){ // BREAKING PHASE
            
            if (dynamics.velocity.z() > breakingDescentRate){
                // to prevent vehicle flipping over when it starts at a velocity greater than breakingDescentRate
                velocitySetpoints.z() = breakingDescentRate;
            }

            velocitySetpoints.y() = correctiveVelocity(currentCrossRange, landingTarget.y(), dynamics.velocity.y());

            if (distToTarget >= stopBreakingBurn){
                velocitySetpoints.x() = breakingVelocity;
            } else{
                velocitySetpoints.x() = correctiveVelocity(currentRange, landingTarget.x(), dynamics.velocity.x());
            }

        } else if (currentAlt >= lockVeclocityAlt){ // APPROACH PHASE
            ignoreRoll = false;

            velocitySetpoints.z() = approachDescentRate;

            // if (fabs(dynamics.velocity.z()) < fabs(approachDescentRate)){

            //     velocitySetpoints.z() = approachDescentRate;
            // }

            velocitySetpoints.y() = correctiveVelocity(currentCrossRange, landingTarget.y(), dynamics.velocity.y());

            if (distToTarget >= stopBreakingBurn){
                velocitySetpoints.x() = approachVelocity;
            } else{
                velocitySetpoints.x() = correctiveVelocity(currentRange, landingTarget.x(), dynamics.velocity.x());
            }

        } else if (currentAlt >= lockOrientationAlt){ // VELOCITY LOCK OUT PHASE
            ignoreRoll = false;
            velocitySetpoints.x() = 0.0;
            velocitySetpoints.y() = 0.0;
            velocitySetpoints.z() = lockVeclocityDescentRate;

        } else { // FINAL DESCENT PHASE
            ignoreRoll = false;
            velocitySetpoints.x() = 0.0;
            velocitySetpoints.y() = 0.0;
            velocitySetpoints.z() = finalDescentRate;
            //maintainOrientation = true;

        }
    }
};

        
        // if (distToTarget >= stopBreakingBurn){ // BREAKING PHASE
        //     velocitySetpoints.x() = breakingVelocity;
        //     velocitySetpoints.y() = correctiveVelocity(currentRange, landingTarget.x(), dynamics.velocity.x());
        //     if (fabs(dynamics.velocity.z()) < fabs(breakingDescentRate)){
        //         velocitySetpoints.z() = breakingDescentRate;
        //     }

        //     if (currentAlt <= secondSlowDownAlt){
        //         velocitySetpoints.z() = initialDescentRate;
        //     }
        // }


    //     // update x velocity setpoint
    //     if ((currentRange - landingTarget.x()) <= correctXrange){
    //         // correct for range to line up with target once we have approached within correctXrange
    //         velocitySetpoints.x() = correctiveVelocity(currentRange, landingTarget.x(), dynamics.velocity.x());
    //     } else{
    //         velocitySetpoints.x() = breakingVelocity;
    //         velocitySetpoints.z() = initialInitialDescentRate;
    //     }
        
    //     // update y velocity setpoint
    //     velocitySetpoints.y() = correctiveVelocity(currentCrossRange, landingTarget.y(), dynamics.velocity.y()); // correct for cross range skew

    //     // update z velocity setpoint
    //     if (distToTarget <= stopDeorbitBurn){ 
    //         // to prevent horizontal overshoot
    //         // later develop logic to handle cases in which the vehicle is comming in at a undershooting trajectory
    //         velocitySetpoints.z() = initialDescentRate;
    //     }
    //     if (currentAlt <= secondSlowDownAlt){
    //             // decrease velocity magnetude further as we approach contact
    //             velocitySetpoints.z() = secondDescentRate;
    //             ignoreRoll = false; // just for fun / to save fuel in early phases / to allow the astronauts a better view of the terrain?
    //         }

    //     // terminal descent conditions
    //     if ((currentAlt <= lockVeclocityAlt) && (currentAlt >= lockOrientationAlt)){
    //         // zero out side velocities in this region
    //         velocitySetpoints.x() = 0.0;
    //         velocitySetpoints.y() = 0.0;
    //         velocitySetpoints.z() = thirdDescentRate;

    //     } else if (currentAlt < lockOrientationAlt){
    //         // keep vehicle upright upon descent and prepare for final descent velocity
    //         maintainOrientation = true;
            
    //         velocitySetpoints.x() = 0.0;
    //         velocitySetpoints.y() = 0.0;
    //         velocitySetpoints.z() = finalDescentRate;
    //     }
    // }




