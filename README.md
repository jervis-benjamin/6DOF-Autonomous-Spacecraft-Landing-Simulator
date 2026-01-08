# Overview 
Simulating the autonomous landing of a spacecraft to meet trajectory and landing criterion, testing controller performance for systems such as thrust vector control (TVC) and reaction control system (RCS), and evaulating guidance algorithms to control various aspects of flight.

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
4) **[Completed]** Integrate controllers for TVC and RCS (full 6DoF control)
5) Add guidance phases and respective control mechanics
6) Transition to LQR controller
7) Implement more advanced controller logic? (ex. H-infinity, MPC)
8) Integrate aerodynamics and test previous controllers
9) Transition from flat world physics to rotating sphere
10) Update visualization tool to accommodate for 3D graphics and rotating spherical world 
11) Integrate controller (PID -> LQR -> MPC)
 
### Resources Referenced  
- Lunar Landing Trajectory Design for Onboard Hazard Detection & Avoidance by Steve Paschall, Tye Brady, Tom Fill Ron Sostaric 
  - Used for simulating Apollo landing trajectory in the sim
 - Apollo Lunar Decent and Ascent Trajectories: https://www.lpi.usra.edu/lunar/documents/Bennett_NASA-TM-X-58040.pdf
   - Used for determining attitude and thrust behavior of Lunar Module from PDI (Powered Decent Initiation) to landing
- Apollo Design Development Documentation: https://ntrs.nasa.gov/api/citations/19780015068/downloads/19780015068.pdf
  - Used for obtaining certain parameters of the Lunar Module