// 6dof Spacecraft Landing Simulator
// By Jervis Benjamin
// Started September 2025

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <fstream>

#include "Spacecraft.h"
#include "Dynamics.h"
#include "Propulsion.h"
#include "World.h"
//#include "Guidance.h"

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
};

int main() {
    World world = World::Moon();
    Spacecraft spacecraft;
    Dynamics dynamics(spacecraft, world);
    Propulsion propulsion(spacecraft, dynamics);
    //Guidance guidance(spacecraft, dynamics, world);

    const double dt = 0.01; // s
    const double tEnd = 600.0; // s
    double t = 0.0; // s

    vector<StateRecord> simData;

    //Eigen::Vector3d testForce(47000.0, 0.0, 0.0); // in the body frame
    Eigen::Vector3d bodyForce = Eigen::Vector3d::Zero();
    double thrust;
    Eigen::Vector3d testTorque(0.0, 0.0, 0.0); // in the body frame

    while ( !(t > tEnd || (dynamics.landed && (dynamics.landingTimer >= dynamics.endTimePost))) ){ // TODO: end sim 5 seconds after landing instead immediately at landing
    //while (t <= tEnd) { 

        /* Main Sim Loop Functions */
        if (dynamics.position.z() < 150.0){
            propulsion.maintainDescentRate = false; // for testing TG phases for guidance
            spacecraft.targetDescentRate = -1.5;
        }else if (dynamics.position.z() < 2000.0){ // roughly start of TG-2
            propulsion.maintainDescentRate = false; 
            spacecraft.targetDescentRate = -20.0;
        } else{
            propulsion.maintainDescentRate = false;
        }
        

        thrust = propulsion.update(dt, 0);
        bodyForce.x() = thrust;
        // tvc and rcs here
        dynamics.update(dt, bodyForce, testTorque);


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
        record.propLevel = 100 * (spacecraft.propellantMass / spacecraft.initialPropellantMass);

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
            cout << "Remaining propellant mass: " << spacecraft.propellantMass << " kg" << endl; 
        }
        
    }


    if (!dynamics.landed){
            cout << "\n\n=== MAX TIME REACHED ===" << endl;
            cout << "Last altitude: " << dynamics.position.z() << " m" << endl;
            cout << "Remaining propellant mass: " << spacecraft.propellantMass << " kg" << endl;
        }
    if (dynamics.tippedOver || (dynamics.landed && (abs(dynamics.impactVelocity) > abs(spacecraft.targetDescentRate)))){ 
            cout << "Spacecraft has crashed into the surface!" << endl;
        }else if (!dynamics.tippedOver && (dynamics.landed && (abs(dynamics.impactVelocity) < abs(spacecraft.targetDescentRate)))){
            cout << "Touchdown confirmed, safe on " << world.name << "!" << endl;
        }
    ofstream outFile("C:/Users/jervi/Documents/projects/6dof Spacecraft Landing Simulator/src/SimVis_tools/simulation_data.csv"); // TODO: fix program such that csv is automatically generated in src
    //ofstream outFile("simulation_data.csv");
    outFile << "time,posX,posY,posZ,velX,velY,velZ,quatW,quatX,quatY,quatZ,omegX,omegY,omegZ,roll,pitch,yaw,totalMass,propMass,x_cg,Ixx,Iyy,Izz,throttleLevel,thrustEngine,propLevel\n";    
    for (const auto& rec : simData) {
        outFile << rec.time << ","
                << rec.position(0) << "," << rec.position(1) << "," << rec.position(2) << ","

                << rec.velocity(0) << "," << rec.velocity(1) << "," << rec.velocity(2) << ","

                << rec.orientation(0) << "," << rec.orientation(1) << "," 
                << rec.orientation(2) << "," << rec.orientation(3) << ","

                << rec.angularVelocity(0) << "," << rec.angularVelocity(1) << "," 
                << rec.angularVelocity(2) << ","

                << rec.eulerAngles_deg(2) << "," << rec.eulerAngles_deg(0) << ","  << rec.eulerAngles_deg(1)  << "," 
                
                << rec.totalMass << "," << rec.propellantMass << ","

                << rec.x_cg << "," << rec.inertia(0,0) << "," << rec.inertia(1,1) << "," << rec.inertia(2,2) << ","
                
                << rec.throttleLevel << "," << rec.thrustEngine << "," << rec.propLevel
                
                << "\n";
    }
    outFile.close();
    cout << "Total steps written: " << simData.size() << endl;
 
    return 0;
}
