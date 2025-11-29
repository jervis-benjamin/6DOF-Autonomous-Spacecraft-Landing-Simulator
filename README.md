# Overview 
Simulating the autonomous 6dof landing of a spacecraft to meet trajectory criterion and testing controller performance for thrust vector control (TVC), reaction control system (RCS), and propulsion.

## Project Breakdown
- Physics:
  - 6dof dynamics to calculate the spacecraft's state over time
  - _Programmed in C++_
- Controls:
  - Logic that takes in state conditions and outputs force values to keep the spacecraft within trajectory and landing contraints
  - _Programmed in C++_  
- Data Representation:
  - Visualizing the spacecraft's trajectory over time
  - _Programmed in Python_

## Current timeline
1) **[Completed]** Create flat world with 6dof dynamics
2) **[Completed]** Develop visualization tool for graphing trajectory
3) **[Completed]** Develop propulsion mechanics (mass depletion, descent rate controller)
4) [_In progress]_ Add guidance phases and respective trajectory mechanics
5) Integrate PID controllers for TVC and RCS (full roll, pitch, and yaw control) + throttling algorithms
6) Transition to LQR controller
7) Implement more advanced controller logic? (ex. H-infinity, MPC)
8) Integrate aerodynamics and test previous controllers
9) Transition from flat world physics to rotating sphere
10) Update visualization tool to accommodate for 3D graphics and rotating spherical world 
11) Integrate controller (PID -> LQR -> MPC)
 
### Resources Referenced
- Lunar Landing Trajectory Design for Onboard Hazard Detection & Avoidance by Steve Paschall, Tye Brady, Tom Fill Ron Sostaric 
  - Used for simulating Apollo landing trajectory in the sim
 - Apollo Lunar Decent and Ascent Trajectories: https://ntrs.nasa.gov/api/citations/19700024568/downloads/19700024568.pdf
   - Used for determining attitude and thrust behavior of Lunar Module from PDI (Powered Decent Initiation) to landing



