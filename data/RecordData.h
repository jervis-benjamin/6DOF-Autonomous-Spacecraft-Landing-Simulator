/*
~ RecordData.h ~

Owns: 
- Simulation_data.bin formatting
*/

#pragma once

#pragma pack(push, 1) // no binary padding

struct RecordData {

    // Note: since we are logging raw binary, it is safer to store values as c++ list instead of using Eigen

    // state variables
    double time;                // [cols: time (s)]
    double position[3];         // [cols: posX (m), posY (m), posZ (m)]
    double velocity[3];         // [cols: velX (m/s), velY (m/s), velZ (m/s)]
    double orientation[4];      // [cols: quatW, quatX, quatY, quatZ]
    double angularVelocity[3];  // [cols: omegX (rad/s), omegY (rad/s), omegZ (rad/s)]
    double eulerAngles_deg[3];      // [cols: pitch (deg), yaw (deg), roll (deg)]

    // mass variables
    double totalMass;           // [cols: totalMass (kg)]
    double propellantMass;      // [cols: propMass (kg)]
    double x_cg;                // [cols: x_cg (m)]
    double inertia[3];     // [cols: Ixx (kg-m^2), Iyy (kg-m^2), Izz (kg-m^2)]

    // propulsion variables
    double throttleLevel;       // [cols: throttleLevel (%)]
    double thrustEngine;        // [cols: thrustEngine (N)]
    double propLevel;           // [cols: propLevel (%)]

    // TVC variables
    double pitchDeflectionDeg;  // [cols: TVC pitch deflection (deg)]
    double yawDeflectionDeg;    // [cols: TVC yaw deflection (deg)]
    double actualThrustVec[3];  // [cols: Thrust in body X (N),Thrust in body Y (N),Thrust in body Z (N)]
    double tvcTorques[3];       // [cols: TVC roll torque (N-m), TVC pitch torque (N-m), TVC yaw torque (N-m)]

    // RCS variables
    int RCS_thrusterSet[3];           // [cols: RCS thrusters state (roll), RCS thrusters state (pitch), RCS thrusters state (yaw)]
    double RCStorques[3];       // [cols: RCS roll torque (N-m), RCS pitch torque (N-m), RCS yaw torque (N-m)]

    // guidance variables
    double velocitySetpoints[3]; // [cols: velX setpoint (m/s), velY setpoint (m/s), VelZ setpoint (m/s)]
    double guidanceState; // [cols: guidanceState]

    // Monte Carlo variables (undispersed values are just default)

};

#pragma pack(pop)