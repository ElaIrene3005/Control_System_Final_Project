# Pole Placement Control for a 3-Link Mechanical System

This project demonstrates the design and implementation of a state-feedback controller using the pole placement method for a simplified 3-link mechanical system. It includes system modeling, controllability analysis, controller design, time-domain simulation, and visualization of the system's dynamic response.

## Overview

The mechanical system consists of three connected links (such as in a robotic arm or multi-pendulum system), each with defined mass and length. The project models the system using a state-space approach and designs a controller to ensure stability and desired performance.

## Objectives

- Model the dynamics of a 3-link mechanical system
- Construct the continuous-time state-space representation
- Analyze system controllability
- Design a pole-placement state feedback controller
- Simulate and visualize the closed-loop system response
- Analyze system stability through eigenvalues

## Features

- Controllability matrix computation and rank verification
- Pole placement using `scipy.signal.place_poles`
- Numerical simulation of system dynamics using `solve_ivp`
- Visualization of state trajectories over time
- Stability analysis via eigenvalue computation

## Methodology

1. **System Parameters**: Mass, length, and gravity are used to build the inertia and gravity matrices.
2. **State-Space Representation**:
   - State vector includes positions and velocities of all three links.
   - Input affects only the third link.
3. **Controllability Analysis**: Construct the controllability matrix and check its rank.
4. **Controller Design**: Desired poles are placed to shape the dynamics.
5. **Simulation**: The system is simulated under the designed feedback using `solve_ivp`.
6. **Visualization**: Time-domain plots of all six states are generated.

## Dependencies

- Python 3.x
- numpy
- scipy
- matplotlib

## Usage

1. Clone this repository.
2. Install the required Python libraries (if not already installed):
   '''
   pip install numpy scipy matplotlib
   '''
3. Run the script:
   '''
   python pole_placement_simulation.py
   '''
