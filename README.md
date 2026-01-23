# Overview 
Simulating the autonomous landing of a spacecraft to meet trajectory and landing criterion, testing controller performance for systems such as thrust vector control (TVC) and reaction control system (RCS), and evaulating guidance algorithms to control various aspects of flight.

## Current timeline (Version 1 Completed)
1) **[Completed]** Create flat world with 6dof dynamics
2) **[Completed]** Develop visualization tool for graphing trajectory
3) **[Completed]** Develop propulsion mechanics (mass depletion, mass properties)
4) **[Completed]** Design and implement controllers for TVC and RCS (full 6DoF control)
5) **[Completed]** Develop guidance algorithm for full control stack functionality
6) **[Completed]** Achieve controlled autonomous landing
7) **[Completed]** Add Monte-Carlo functionality

### Tentative Additons
  - Transition from flat world physics to rotating sphere
  - Implement aerodynamics
  - Implement more advanced controller laws (ex. LQR, H-infinity, MPC)

## Resources Referenced  
- Lunar Lander model obtained from NASA 3D Resources
- Lunar Landing Trajectory Design for Onboard Hazard Detection & Avoidance by Steve Paschall, Tye Brady, Tom Fill Ron Sostaric 
  - Used for simulating Apollo landing trajectory in the sim
 - Apollo Lunar Decent and Ascent Trajectories: https://www.lpi.usra.edu/lunar/documents/Bennett_NASA-TM-X-58040.pdf
   - Used for determining attitude and thrust behavior of Lunar Module from PDI (Powered Decent Initiation) to landing
- Apollo Design Development Documentation: https://ntrs.nasa.gov/api/citations/19780015068/downloads/19780015068.pdf
  - Used for obtaining certain parameters of the Lunar Module