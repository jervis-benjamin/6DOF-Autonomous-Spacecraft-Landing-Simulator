// 6dof Spacecraft Landing Simulator
// By Jervis Benjamin
// Started September 2025

/*

TODOs:
- add tvc and rcs stats to simvis3d and fix plume not showing up for some reason
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

#include "../data/RecordData.h"

using namespace std;

int main() {
    World world = World::Moon();
    Spacecraft spacecraft;
    Dynamics dynamics(spacecraft, world);
    Guidance guidance(spacecraft, dynamics, world);
    Propulsion propulsion(spacecraft, dynamics, world);
    TVC_Controller tvc(spacecraft, dynamics, propulsion);
    RCS_Controller rcs(spacecraft, dynamics);

    // simulation settings
    const double dt = 0.01; // s
    const double tEnd = 1200.0; // s
    double t = 0.0; // s

    // data recording setup
    ofstream outFile("C:/Users/jervi/Documents/projects/6dof Spacecraft Landing Simulator/data/simulation_data.bin", ios::binary);
    //ofstream outFile("simulation_data.bin", ios::binary);

    if (!outFile.is_open()) {
        cerr << "Error: could not open simulation_data.bin" << endl;
        return -1;
    }

    cout << "Run started...\n" << endl;


    /* Test Inputs */
    // Eigen::Vector3d testForce(47000.0, 0.0, 0.0); // in the body frame
    // Eigen::Vector3d testTorque(0.0, 0.0, 0.0); // in the body frame

    /* Body forces */
    Eigen::Vector3d bodyForces = Eigen::Vector3d::Zero();
    Eigen::Vector3d bodyTorques = Eigen::Vector3d::Zero();

    while ( !(t > tEnd || (dynamics.landed && (dynamics.landingTimer >= dynamics.endTimePost))) ) { 
    //while (t <= tEnd) { 

        /* Main Sim Loop Functions */

        // reset net forces and torques
        bodyForces = Eigen::Vector3d::Zero();
        bodyTorques = Eigen::Vector3d::Zero();

        // update guidance
        guidance.update();

        // update propulsion and get command thrust vector
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
        /*
        FORMAT in RecordData.h:
        type varName; // [cols: Name_units]
        */
        RecordData rec;
        rec.time = t;

        rec.position[0] = dynamics.position.x();
        rec.position[1] = dynamics.position.y();
        rec.position[2] = dynamics.position.z();

        rec.velocity[0] = dynamics.velocity.x();
        rec.velocity[1] = dynamics.velocity.y();
        rec.velocity[2] = dynamics.velocity.z();

        rec.orientation[0] = dynamics.orientation(0);
        rec.orientation[1] = dynamics.orientation(1);
        rec.orientation[2] = dynamics.orientation(2);
        rec.orientation[3] = dynamics.orientation(3);

        rec.angularVelocity[0] = dynamics.angularVelocity.x();
        rec.angularVelocity[1] = dynamics.angularVelocity.y();
        rec.angularVelocity[2] = dynamics.angularVelocity.z();

        Eigen::Vector3d eulerAngles_deg = QuaternionTools::toEulerAngles(QuaternionTools::toWorld(dynamics.orientation));
        rec.eulerAngles_deg[0] = eulerAngles_deg(0);
        rec.eulerAngles_deg[1] = eulerAngles_deg(1);
        rec.eulerAngles_deg[2] = eulerAngles_deg(2);

        rec.totalMass = spacecraft.totalMass;

        rec.propellantMass = spacecraft.propellantMass;

        rec.x_cg = spacecraft.cg.x();

        rec.inertia[0] = spacecraft.inertia(0,0);
        rec.inertia[1] = spacecraft.inertia(1,1);
        rec.inertia[2] = spacecraft.inertia(2,2);

        rec.throttleLevel = propulsion.throttleLevel;
        rec.thrustEngine = propulsion.thrustEngine;
        rec.propLevel = (spacecraft.propellantMass / spacecraft.initialPropellantMass) * 100;

        rec.pitchDeflectionDeg = tvc.pitchDeflectionRad;
        rec.yawDeflectionDeg = tvc.yawDeflectionRad;

        rec.actualThrustVec[0] = tvc.actualThrustVector.x();
        rec.actualThrustVec[1] = tvc.actualThrustVector.y();
        rec.actualThrustVec[2] = tvc.actualThrustVector.z();

        rec.tvcTorques[0] = tvc.TVCtorques.x();
        rec.tvcTorques[1] = tvc.TVCtorques.y();
        rec.tvcTorques[2] = tvc.TVCtorques.z();

        rec.RCS_thrusterSet[0] = static_cast<double>(rcs.RCS_thrusterSet[0]);
        rec.RCS_thrusterSet[1] = static_cast<double>(rcs.RCS_thrusterSet[1]);
        rec.RCS_thrusterSet[2] = static_cast<double>(rcs.RCS_thrusterSet[2]);

        rec.RCStorques[0] = rcs.RCStorques.x();
        rec.RCStorques[1] = rcs.RCStorques.y();
        rec.RCStorques[2] = rcs.RCStorques.z();

        rec.velocitySetpoints[0] = guidance.velocitySetpoints.x();
        rec.velocitySetpoints[1] = guidance.velocitySetpoints.y();
        rec.velocitySetpoints[2] = guidance.velocitySetpoints.z();

        rec.guidanceState = guidance.guidanceState;

        outFile.write(reinterpret_cast<const char*>(&rec), sizeof(RecordData));

        t += dt; // update time

        if (dynamics.landed && !(dynamics.landingTimer >= dynamics.endTimePost)){
            dynamics.landingTimer += dt;
        }
        if(dynamics.landed && (dynamics.landingTimer >= dynamics.endTimePost)){
            cout << "\n=== CONTACT DETECTED ===" << endl;
            cout << "Time at contact: " << t - dynamics.landingTimer << " s" << endl;
            cout << "Final velocity: " << dynamics.impactVelocity << " m/s" << endl;
            cout << "Remaining propellant mass: " << spacecraft.propellantMass << " kg (" << (spacecraft.propellantMass / spacecraft.initialPropellantMass) * 100 << " %)" << endl;
            cout << "Last position relative to target (m): " << dynamics.position.transpose() << endl; 
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
    

    outFile.close();
    cout << "\n\nRun finished, data saved to simulation_data.bin" << endl;
    cout << "Converting data to parquet..." << endl;

    string pythonExe = "C:/Users/jervi/anaconda3/python.exe";
    string scriptPath = "C:/Users/jervi/Documents/projects/6dof Spacecraft Landing Simulator/data/BinToParquet.py";

    // create one big python command in case we move BinToParquet.py or python gets picky again
    string command = pythonExe + " \"" + scriptPath + "\"";

    // communicate possible errors to terminal
    int result = system(command.c_str());
    if (result == 0) {
        cout << "simulation_data.bin.parquet created successfully." << endl;
    } else {
        cout << "ERROR: Python script failed. Return code: " << result << endl;
    }

    return 0;
 
}
