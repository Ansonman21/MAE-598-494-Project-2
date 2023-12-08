# Rocket Landing Optimization Using Model Predictive Control (MPC)

## Project Overview

This project focuses on optimizing the fuel consumption of a rocket during its landing phase using Model Predictive Control (MPC). The primary goal is to safely land the rocket from a certain height while minimizing the amount of fuel used. This is achieved by dynamically adjusting the rocket's thrust over time based on the current state (height and velocity) and the desired final state (landing on the ground with minimal velocity).

## Technical Details

### Dynamics

The rocket's dynamics are modeled in a simplified 1D environment, considering only the vertical motion. The key factors in the dynamics include gravity, thrust, and the rocket's mass. The dynamics are governed by the following equations:

- Acceleration: `a = (thrust - gravity) / mass`
- Velocity: `v = v0 + a * dt`
- Position: `y = y0 + v0 * dt + 0.5 * a * dt^2`

where `dt` is the time step, `v0` and `y0` are the initial velocity and position, respectively.

### Model Predictive Control (MPC)

MPC is used to optimize the control inputs (thrust) at each time step. The optimization problem is set up to minimize the total thrust used while ensuring the rocket lands safely. The constraints ensure that the thrust is within allowable limits and that the rocket does not go below the ground level.

### Implementation

The project is implemented in Python, utilizing libraries such as NumPy for numerical operations and CVXPY for solving the optimization problem. The simulation runs over a predefined number of time steps, and at each step, the MPC problem is solved to determine the optimal thrust. The rocket's state is then updated based on the dynamics and the control input.

### Visualization

The results are visualized using Matplotlib, showing the trajectory of the rocket in terms of its height and velocity over time, as well as the thrust applied at each time step.

## Usage

To run the simulation, execute the provided Python script. The script will perform the MPC-based optimization for the rocket landing and display the results in a series of plots.

## Dependencies

- Python 3.x
- NumPy
- CVXPY
- Matplotlib

## Conclusion

This project demonstrates the application of MPC in a practical scenario of rocket landing. The optimization approach ensures efficient fuel usage while adhering to the constraints of safe landing, showcasing the potential of MPC in aerospace engineering and control systems.
