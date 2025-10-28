// 6dof Spacecraft Landing Simulator
// By Jervis Benjamin
// Started September 2025

/*

TODOs:
- integrate mass depletion and thrust mechanics

*/

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <fstream>

#include "SpacecraftConfig.h"
#include "Dynamics.h"
#include "World.h"

using namespace std;

struct StateRecord {
    double time;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector4d orientation;
    Eigen::Vector3d angularVelocity;
    Eigen::Vector3d eulerAngles_deg;
};

int main() {
    World world = World::Earth();
    SpacecraftConfig config;
    Dynamics dynamics(config, world);

    const double dt = 0.01; // s
    const double tEnd = 30.0; // s
    double t = 0.0; // s

    vector<StateRecord> simData;

    Eigen::Vector3d testForce(50000.0, 0.0, 0.0); // in the body frame
    Eigen::Vector3d testTorque(0.0, 0.0, 0.0); // in the body frame

    //while (t <= tEnd && !dynamics.landed) {
    while (t <= tEnd) {    
        dynamics.update(dt, testForce, testTorque);

        StateRecord record;
        record.time = t;
        record.position = dynamics.position;
        record.velocity = dynamics.velocity;
        record.orientation = dynamics.orientation;
        record.angularVelocity = dynamics.angularVelocity;
        record.eulerAngles_deg = QuaternionTools::toEulerAngles(dynamics.orientation);

        simData.push_back(record);

        cout << "Time: " << t
             << " [s] | Pos: " << dynamics.position.transpose()
             << " [m] | Vel: " << dynamics.velocity.transpose()

             << " [m/s] | Ori: (" << dynamics.orientation(0) << ", " << dynamics.orientation(1) << ", "
             << dynamics.orientation(2) << ", " << dynamics.orientation(3) << ")"

             << " | RolPitYaw: (" << record.eulerAngles_deg(2) << ", " << record.eulerAngles_deg(0) << ", "
             << record.eulerAngles_deg(1) <<  ")"

             << " [deg] | AngVel: " << dynamics.angularVelocity.transpose()
             << endl;

        t += dt;

        if (dynamics.landed){
            cout << "\n\n=== LANDING DETECTED ===" << endl;
            cout << "Time at landing: " << t << " s" << endl;
            cout << "Final speed: " << dynamics.impactVelocity << " m/s" << endl;
            }
    }

    if (!dynamics.landed){
            cout << "\n\n=== MAX TIME REACHED ===" << endl;
            cout << "Last altitude: " << dynamics.position.z() << " m" << endl;
            }

    ofstream outFile("C:/Users/jervi/Documents/projects/6dof Spacecraft Landing Simulator/src/SimVis_tools/simulation_data.csv"); // TODO: fix program such that csv is automatically generated in src
    //ofstream outFile("simulation_data.csv");
    outFile << "time,posX,posY,posZ,velX,velY,velZ,quatW,quatX,quatY,quatZ,omegX,omegY,omegZ,roll,pitch,yaw\n";    
    for (const auto& rec : simData) {
        outFile << rec.time << ","
                << rec.position(0) << "," << rec.position(1) << "," << rec.position(2) << ","

                << rec.velocity(0) << "," << rec.velocity(1) << "," << rec.velocity(2) << ","

                << rec.orientation(0) << "," << rec.orientation(1) << "," 
                << rec.orientation(2) << "," << rec.orientation(3) << ","

                << rec.angularVelocity(0) << "," << rec.angularVelocity(1) << "," 
                << rec.angularVelocity(2) << ","

                << rec.eulerAngles_deg(2) << "," << rec.eulerAngles_deg(0) << ","  << rec.eulerAngles_deg(1) << "\n";
    }
    outFile.close();
    cout << "Total steps written: " << simData.size() << endl;
 
    return 0;
}
