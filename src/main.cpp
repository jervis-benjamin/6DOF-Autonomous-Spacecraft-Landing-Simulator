// 6dof Spacecraft Landing Simulator
// By Jervis Benjamin
// Started September 2025

/*

TODOs:
- graph 3 sigma landing elipse
- add IMU class (on and off errors) and add TVC and RCS errors

- make sure to have descriptions for all functions (be more diligent on best practices)
- (low priority) make all class functions optimized for speed (similar to how it is in Dynamics.h)

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

#include "../include/RunConfig.h"

#include "../data/RecordData.h"

using namespace std;

int main() {
    World world = World::Moon();
    RunConfig runConfig; // configure monte carlo dispersions in RunConfig.h
    int runs = 1;
    const double dt = 0.01; // s
    const double tEnd = 1200.0; // s
    double t; // s
    Eigen::Vector3d bodyForces;
    Eigen::Vector3d bodyTorques;

    // file paths
    string pythonExe = "C:/Users/jervi/anaconda3/python.exe";
    string BinToParquet = "C:/Users/jervi/Documents/projects/6dof Spacecraft Landing Simulator/data/BinToParquet.py";
    string MonteCarloCleaner = "C:/Users/jervi/Documents/projects/6dof Spacecraft Landing Simulator/data/MonteCarloCleaner.py";

    // if we are running Monte Carlos, clean the data folder so new generated files don't get mixed up with old ones
    if(runConfig.monteCarloRun){
        cout << "\nCleaning previous Monte Carlo data to prepare for new data.." << endl;

        string command = pythonExe + " \"" + MonteCarloCleaner + "\"";

        int result = system(command.c_str());

        if (result == 0) {
            cout << "Cleaning complete!" << endl;
        } else {
            cout << "ERROR: Python script failed. Return code: " << result << endl;
        }
        cout << endl;
    }





    while (runs <= runConfig.runNum){

        /* 
        ======================================
        |        Configure Data File         |
        ======================================
        */

        cout << "============================================\n" << endl;

        string fileName;
        if (runConfig.monteCarloRun){
            fileName = "C:/Users/jervi/Documents/projects/6dof Spacecraft Landing Simulator/data/monte_carlo/mc_run_" + to_string(runs) + ".bin";
        }else{
            fileName = "C:/Users/jervi/Documents/projects/6dof Spacecraft Landing Simulator/data/simulation_data.bin";
        }
        ofstream outFile(fileName, ios::binary);

        if (!outFile.is_open()) {
            cerr << "Error: could not open simulation_data.bin" << endl;
            return -1;
        }

        cout << "Run ";
        if (runConfig.monteCarloRun){
            cout << runs;
        }
        cout << " started...\n" << endl;






        /* 
        ==============================================
        |        Initialize Simulation Values        |
        ==============================================
        */

        // reset time
        t = 0.0;

        // reset body forces and torques
        bodyForces = Eigen::Vector3d::Zero();
        bodyTorques = Eigen::Vector3d::Zero();

        // only get dispersions once per loop iteration
        if (runConfig.monteCarloRun){
            runConfig.getDispersions();
        }

        // initializing objects following order of root dependancy (Spacecraft -> Dynamics -> Guidance -> Propulsion -> TVC_Controller -> RCS_Controller)
        Spacecraft spacecraft; 

        // spacecraft dispersions
        if (runConfig.monteCarloRun) {

            spacecraft.propellantMass *= runConfig.simInputs.initialPropMass_mult;
            spacecraft.initialPropellantMass = spacecraft.propellantMass;
            spacecraft.updateMassProperties();

            spacecraft.initialPosition.x() += runConfig.simInputs.initialPosX_add;
            spacecraft.initialPosition.y() += runConfig.simInputs.initialPosY_add;
            spacecraft.initialPosition.z() += runConfig.simInputs.initialPosZ_add;
            
        }

        Dynamics dynamics(spacecraft, world);
        Guidance guidance(spacecraft, dynamics, world);
        Propulsion propulsion(spacecraft, dynamics, world);
        TVC_Controller tvc(spacecraft, dynamics, propulsion);
        RCS_Controller rcs(spacecraft, dynamics);

        // initialize input variables (Monte Carlo dispersed values)
        double mc_initialPropMass = spacecraft.initialPropellantMass;
        double mc_initialPosX = spacecraft.initialPosition.x();
        double mc_initialPosY = spacecraft.initialPosition.y();
        double mc_initialPosZ = spacecraft.initialPosition.z();





        /* 
        =========================================
        |        Main Sim Loop Functions        |
        =========================================
        */

        while ( !(t > tEnd || (dynamics.landed && (dynamics.landingTimer >= dynamics.endTimePost))) ) { 

            // reset net forces and torques
            bodyForces = Eigen::Vector3d::Zero();
            bodyTorques = Eigen::Vector3d::Zero();

            // update guidance
            guidance.update(dt);

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




            /* 
            ====================================
            |        Recording Sim Data        |
            ====================================
            */
            
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

            rec.throttleLevel = propulsion.throttleLevel * 100; // %
            rec.thrustEngine = propulsion.thrustEngine;
            rec.propLevel = (spacecraft.propellantMass / spacecraft.initialPropellantMass) * 100; // %

            rec.pitchDeflectionDeg = tvc.pitchDeflectionRad * (180/PI);
            rec.yawDeflectionDeg = tvc.yawDeflectionRad * (180/PI);

            rec.actualThrustVec[0] = tvc.actualThrustVector.x();
            rec.actualThrustVec[1] = tvc.actualThrustVector.y();
            rec.actualThrustVec[2] = tvc.actualThrustVector.z();

            rec.tvcTorques[0] = tvc.TVCtorques.x();
            rec.tvcTorques[1] = tvc.TVCtorques.y();
            rec.tvcTorques[2] = tvc.TVCtorques.z();

            rec.RCS_thrusterSet[0] = rcs.RCS_thrusterSet[0];
            rec.RCS_thrusterSet[1] = rcs.RCS_thrusterSet[1];
            rec.RCS_thrusterSet[2] = rcs.RCS_thrusterSet[2];

            rec.RCStorques[0] = rcs.RCStorques.x();
            rec.RCStorques[1] = rcs.RCStorques.y();
            rec.RCStorques[2] = rcs.RCStorques.z();

            rec.velocitySetpoints[0] = guidance.velocitySetpoints.x();
            rec.velocitySetpoints[1] = guidance.velocitySetpoints.y();
            rec.velocitySetpoints[2] = guidance.velocitySetpoints.z();

            rec.guidanceState = guidance.guidanceState;

            rec.mc_initialPropMass = mc_initialPropMass;

            rec.mc_initialPos[0] = mc_initialPosX;
            rec.mc_initialPos[1] = mc_initialPosY;
            rec.mc_initialPos[2] = mc_initialPosZ;

            /*
            maybe add initial dispersed value as more columns
            */

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
                //cout << "Last position relative to target (m): " << dynamics.position.transpose() << endl; 
                cout << "Last position relative to target (m): " << dynamics.position.x() << " "  << dynamics.position.y() << " "  << dynamics.position.z() << endl;
            }
        }




        /* 
        ====================================
        |        End of Run Summary        |
        ====================================
        */
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

        // if monteCarloRun is false, end the loop
        if (!runConfig.monteCarloRun){
            runConfig.runNum = 1; // runs is initialized to 1 at the start so this will stop the while loop
        }




        // update run count
        runs++;
        outFile.close();
        cout << "\nRun finished, data saved." << endl;
    }

    
    cout << "Converting data to parquet..." << endl;
    
    string command = pythonExe + " \"" + BinToParquet + "\"";

    int result = system(command.c_str());
    if (result == 0) {
        cout << ".parquet created successfully." << endl;
    } else {
        cout << "ERROR: Python script failed. Return code: " << result << endl;
    }

    return 0;
}
