// 6dof Spacecraft Landing Simulator
// By Jervis Benjamin
// Started September 1st 2025

/*

Major TODOs:

Physics:
- make the world/planet round
- integrate mass depletion and fix angular velocity calcs to account for changing MoI
- verify what frames values are in reference to (for example, if omega is in body or inertial)

Controls:
- integrate controller

SimVis:
- Create 3d viewer

*/


#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <fstream>

#include "SpacecraftConfig.h"
#include "Dynamics.h"

using namespace std;

struct StateRecord {
    double time;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector4d orientation;
    Eigen::Vector3d angularVelocity;
};

int main() {
    SpacecraftConfig config;
    Dynamics dynamics(config);

    const double dt = 0.01;
    const double tEnd = 10.0;
    double t = 0.0;

    vector<StateRecord> simData;

    Eigen::Vector3d testForce(0, 0, 0);
    Eigen::Vector3d testTorque(0, 0, 0);

    while (t <= tEnd) {
        dynamics.update(dt, testForce, testTorque);

        StateRecord record;
        record.time = t;
        record.position = dynamics.position;
        record.velocity = dynamics.velocity;
        record.orientation = dynamics.orientation;
        record.angularVelocity = dynamics.angularVelocity;

        simData.push_back(record);

        cout << "Time: " << t
             << " Pos: " << dynamics.position.transpose()
             << " Vel: " << dynamics.velocity.transpose()
             << " Ori: (" << dynamics.orientation(0) << ", " << dynamics.orientation(1) << ", "
             << dynamics.orientation(2) << ", " << dynamics.orientation(3) << ")"
             << " AngVel: " << dynamics.angularVelocity.transpose()
             << endl;

        t += dt;
    }

    ofstream outFile("C:/Users/jervi/Documents/projects/6dof Spacecraft Landing Simulator/src/simulation_data.csv"); // TODO: fix program such that csv is automatically generated in src
    //ofstream outFile("simulation_data.csv");
    outFile << "time,posX,posY,posZ,velX,velY,velZ,quatW,quatX,quatY,quatZ,omegX,omegY,omegZ\n";    
    for (const auto& rec : simData) {
        outFile << rec.time << ","
                << rec.position(0) << "," << rec.position(1) << "," << rec.position(2) << ","
                << rec.velocity(0) << "," << rec.velocity(1) << "," << rec.velocity(2) << ","
                << rec.orientation(0) << "," << rec.orientation(1) << "," 
                << rec.orientation(2) << "," << rec.orientation(3) << ","
                << rec.angularVelocity(0) << "," << rec.angularVelocity(1) << "," 
                << rec.angularVelocity(2) << "\n";
    }
    outFile.close();
    cout << "Total steps written: " << simData.size() << endl;
 
    return 0;
}
