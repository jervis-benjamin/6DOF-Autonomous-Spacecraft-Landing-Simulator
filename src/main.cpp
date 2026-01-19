// 6dof Spacecraft Landing Simulator
// By Jervis Benjamin
// Started September 2025

/*

TODOs:
- make the .h and .cpp split
- add tvc and rcs stats to simvis3d
- find faster way to log data into a csv
- add IMU class (on and off errors)
- add monte carlo feature and look into what to disperse
- graph 3 sigma landing elipse

- make sure to have descriptions for all functions (be more diligent on best practices)
- (low priority) make all class functions optimized for speed (similar to how it is in Dynamics.h)
- long term goal: implement monte-carlo ability and find ways to optimize sim for speed

*/

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <fstream>

#include "../include/Spacecraft.h"
#include "../include/Dynamics.h"
#include "../include/World.h"
#include "../include/Guidance.h"
#include "../include/Propulsion.h"
#include "../include/TVC_Controller.h"
#include "../include/RCS_Controller.h"

using namespace std;

struct StateRecord {
    double time;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector4d orientation;
    Eigen::Vector3d angularVelocity;
    Eigen::Vector3d eulerAngles_deg; // following pitch-yaw-roll
    double totalMass;
    double propellantMass;
    double x_cg;
    Eigen::Matrix3d inertia;
    double throttleLevel;
    double thrustEngine;
    double propLevel;
    double pitchDeflectionDeg;
    double yawDeflectionDeg;
    Eigen::Vector3d actualThrustVector;
    Eigen::Vector3d TVCtorques;
    array<int, 3> RCS_thrusterSet;
    Eigen::Vector3d RCStorques;
    Eigen::Vector3d velocitySetpoints;
    double guidanceState;
};

int main() {
    World world = World::Moon();
    Spacecraft spacecraft;
    Dynamics dynamics(spacecraft, world);
    Guidance guidance(spacecraft, dynamics, world);
    Propulsion propulsion(spacecraft, dynamics, world);
    TVC_Controller tvc(spacecraft, dynamics, propulsion);
    RCS_Controller rcs(spacecraft, dynamics);

    const double dt = 0.01; // s
    const double tEnd = 1200.0; // s
    double t = 0.0; // s

    vector<StateRecord> simData;

    /* Test Inputs */
    // Eigen::Vector3d testForce(47000.0, 0.0, 0.0); // in the body frame
    // Eigen::Vector3d testTorque(0.0, 0.0, 0.0); // in the body frame

    /* Body forces */
    Eigen::Vector3d bodyForces = Eigen::Vector3d::Zero();
    Eigen::Vector3d bodyTorques = Eigen::Vector3d::Zero();

    while ( !(t > tEnd || (dynamics.landed && (dynamics.landingTimer >= dynamics.endTimePost))) ){ 
    //while (t <= tEnd) { 

        /* Main Sim Loop Functions */

        // reset net forces and torques
        bodyForces = Eigen::Vector3d::Zero();
        bodyTorques = Eigen::Vector3d::Zero();

        // update guidance
        guidance.update();
        // Eigen::Vector3d testSetpoints = dynamics.velocity;
        // testSetpoints.z() = -50;

        // update propulsion and get command thrust vector
        // propulsion.update(dt, testSetpoints);
        propulsion.update(dt, guidance.velocitySetpoints);
        propulsion.updateMassFromEngine(dt);

        // run TVC controller
        tvc.runTVC(dt, propulsion.thrustEngine, propulsion.idealthrustDirection);
        bodyForces += tvc.actualThrustVector;
        bodyTorques += tvc.TVCtorques;

        // run RCS controller
        rcs.runRCS(dt, propulsion.idealthrustDirection, guidance.ignoreRoll, guidance.rollSetpointDeg, guidance.eulerSetpointsDeg, guidance.maintainOrientation);
        rcs.updateMassFromRCS(dt);
        bodyTorques += rcs.RCStorques;

        // update state with inputs
        dynamics.update(dt, bodyForces, bodyTorques);

        // update spacecraft mass
        spacecraft.updateMassProperties();

        /* Recording Sim Data*/
        StateRecord record;
        record.time = t;
        record.position = dynamics.position;
        record.velocity = dynamics.velocity;
        record.orientation = dynamics.orientation;
        record.angularVelocity = dynamics.angularVelocity;
        record.eulerAngles_deg = QuaternionTools::toEulerAngles(QuaternionTools::toWorld(dynamics.orientation));
        record.totalMass = spacecraft.totalMass;
        record.propellantMass = spacecraft.propellantMass;
        record.x_cg = spacecraft.cg.x();
        record.inertia = spacecraft.inertia;
        record.throttleLevel = propulsion.throttleLevel * 100.0;
        record.thrustEngine = propulsion.thrustEngine;
        record.propLevel = (spacecraft.propellantMass / spacecraft.initialPropellantMass) * 100;
        record.pitchDeflectionDeg = tvc.pitchDeflectionRad * (180/PI);
        record.yawDeflectionDeg = tvc.yawDeflectionRad * (180/PI);
        record.actualThrustVector = tvc.actualThrustVector;
        record.TVCtorques = tvc.TVCtorques;
        record.RCS_thrusterSet = rcs.RCS_thrusterSet;
        record.RCStorques = rcs.RCStorques;
        record.velocitySetpoints = guidance.velocitySetpoints;
        record.guidanceState = guidance.guidanceState;

        simData.push_back(record);

        /*
        cout << "Time: " << t
             << " [s] | Pos: " << dynamics.position.transpose()
             << " [m] | Vel: " << dynamics.velocity.transpose()

             << " [m/s] | Ori: (" << dynamics.orientation(0) << ", " << dynamics.orientation(1) << ", "
             << dynamics.orientation(2) << ", " << dynamics.orientation(3) << ")"

             << " | RolPitYaw: (" << record.eulerAngles_deg(2) << ", " << record.eulerAngles_deg(0) << ", "
             << record.eulerAngles_deg(1) <<  ")"

             << " [deg] | AngVel: " << dynamics.angularVelocity.transpose()
             << endl;
        */

        t += dt; // update time

        if (dynamics.landed && !(dynamics.landingTimer >= dynamics.endTimePost)){
            dynamics.landingTimer += dt;
        }
        if(dynamics.landed && (dynamics.landingTimer >= dynamics.endTimePost)){
            cout << "\n\n=== CONTACT DETECTED ===" << endl;
            cout << "Time at contact: " << t - dynamics.landingTimer << " s" << endl;
            cout << "Final velocity: " << dynamics.impactVelocity << " m/s" << endl;
            cout << "Remaining propellant mass: " << spacecraft.propellantMass << " kg (" << (spacecraft.propellantMass / spacecraft.initialPropellantMass) * 100 << " %)" << endl; 
        }
        
    }


    if (!dynamics.landed){
        cout << "\n\n=== MAX TIME REACHED ===" << endl;
        cout << "Last altitude: " << dynamics.position.z() << " m" << endl;
        cout << "Remaining propellant: " << (spacecraft.propellantMass / spacecraft.initialPropellantMass) * 100 << "%" << endl;
    }
    if (dynamics.tippedOver || (dynamics.landed && (abs(dynamics.impactVelocity) > abs(spacecraft.touchdownVelocityLimit)))){ 
        cout << "Spacecraft has crashed into the surface!" << endl;
        cout << "\nFaliure due to:" << endl;
        if (dynamics.tippedOver){
            cout << "— Exceeding tip-over angle safety limit" << endl;
        }
        if (abs(dynamics.impactVelocity) > abs(spacecraft.touchdownVelocityLimit)){
            cout << "— Exceeding touchdown velocity limit" << endl;
        }
    }else if (!dynamics.tippedOver && (dynamics.landed && (abs(dynamics.impactVelocity) < abs(spacecraft.touchdownVelocityLimit)))){
        cout << "Touchdown confirmed, safe on " << world.name << "!" << endl;
    }
    cout << "\nLast position relative to target (m): " << dynamics.position.transpose() << endl;

    ofstream outFile("C:/Users/jervi/Documents/projects/6dof Spacecraft Landing Simulator/visualization/simulation_data.csv"); // TODO: fix program such that csv is automatically generated in folder
    //ofstream outFile("simulation_data.csv");
    outFile << "time (s),posX (m),posY (m),posZ (m),velX (m/s),velY (m/s),velZ (m/s),quatW,quatX,quatY,quatZ,omegX (rad/s),omegY (rad/s),omegZ (rad/s),pitch (deg),yaw (deg),roll (deg),totalMass (kg),propMass (kg),x_cg (m),Ixx (kg-m^2),Iyy (kg-m^2),Izz (kg-m^2),throttleLevel (%),thrustEngine (N),propLevel (%),TVC pitch deflection (deg),TVC yaw deflection (deg),Thrust in body X (N),Thrust in body Y (N),Thrust in body Z (N),TVC roll torque (N-m),TVC pitch torque (N-m),TVC yaw torque (N-m),RCS thrusters state (roll),RCS thrusters state (pitch),RCS thrusters state (yaw),RCS roll torque (N-m),RCS pitch torque (N-m),RCS yaw torque (N-m),velX setpoint (m/s),velY setpoint (m/s),velZ setpoint (m/s),guidanceState\n";    
    for (const auto& rec : simData) {
        outFile << rec.time << ","
                << rec.position(0) << "," << rec.position(1) << "," << rec.position(2) << ","

                << rec.velocity(0) << "," << rec.velocity(1) << "," << rec.velocity(2) << ","

                << rec.orientation(0) << "," << rec.orientation(1) << "," 
                << rec.orientation(2) << "," << rec.orientation(3) << ","

                << rec.angularVelocity(0) << "," << rec.angularVelocity(1) << "," 
                << rec.angularVelocity(2) << ","

                << rec.eulerAngles_deg(0) << "," << rec.eulerAngles_deg(1) << ","  << rec.eulerAngles_deg(2)  << "," 
                
                << rec.totalMass << "," << rec.propellantMass << ","

                << rec.x_cg << "," << rec.inertia(0,0) << "," << rec.inertia(1,1) << "," << rec.inertia(2,2) << ","
                
                << rec.throttleLevel << "," << rec.thrustEngine << "," << rec.propLevel << ","

                << rec.pitchDeflectionDeg << "," << rec.yawDeflectionDeg << ","

                << rec.actualThrustVector(0) << "," << rec.actualThrustVector(1) << "," << rec.actualThrustVector(2) << ","

                << rec.TVCtorques(0) << "," << rec.TVCtorques(1) << "," << rec.TVCtorques(2) << ","

                << rec.RCS_thrusterSet[0] << "," << rec.RCS_thrusterSet[1] << "," << rec.RCS_thrusterSet[2] << ","
                
                << rec.RCStorques(0) << "," << rec.RCStorques(1) << "," << rec.RCStorques(2) << ","
                
                << rec.velocitySetpoints.x() << "," << rec.velocitySetpoints.y() << "," << rec.velocitySetpoints.z() << ","

                << rec.guidanceState

                << "\n";
    }
    outFile.close();
    cout << "\nTotal steps written: " << simData.size() << endl;
 
    return 0;
}
