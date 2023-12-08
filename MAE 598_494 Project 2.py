import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

# Constants
G = 9.81  # Gravity (m/s^2)
V_e = 2570  # Exit velocity at nozzle (m/s)
m_0 = 28122.7269  # Total initial rocket mass (kg)
FRAME_TIME = 0.1  # Time interval
T = 200  # MPC horizon
total_time_step = 200  # Total simulation time steps

initial_height = 200  # Initial height of rocket (m)
initial_velocity = 0  # Initial velocity of rocket (m/s)
initial_state = np.array([initial_height, initial_velocity, 0, 0, 0])  # Example initial state

# Dynamics function
def rocket_dynamics(state, m_dot_p):
    y, vy = state
    a_r = (m_dot_p * V_e - m_0 * G) / m_0
    vy_new = vy + a_r * FRAME_TIME
    y_new = y + vy * FRAME_TIME + 0.5 * a_r * FRAME_TIME ** 2
    return np.array([y_new, vy_new])

# MPC Function
def mpc(state, T):
    n_state = 2  # Height and velocity
    n_control = 1  # Thrust
    max_thrust = 1000  # Maximum thrust value

    # Optimization variables
    x = cp.Variable((n_state, T + 1))
    u = cp.Variable((n_control, T))

    # Dynamics matrices
    A = np.array([[1, FRAME_TIME], [0, 1]])  # State transition matrix
    B = np.array([[0], [FRAME_TIME]])  # Control input matrix

    # Constraints
    print(state)
    constraints = [x[:, 0] == [199.95095, -0.981]]  # State is already a 1D array with 2 elements
    for t in range(T):
        constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t]]
        constraints += [u[0, t] >= 0, u[0, t] <= max_thrust]

    # Objective function to minimize fuel usage
    objective = cp.Minimize(cp.sum_squares(u))

    # Define and solve the problem
    problem = cp.Problem(objective, constraints)
    problem.solve()

    if problem.status != cp.OPTIMAL:
        return None
    return u.value[0]

# Control loop
state_trajectory = []
action_trajectory = []
state = np.array([initial_height, initial_velocity])

for i in range(total_time_step):
    control_action = mpc(state, T - i)
    if control_action is None:
        print('MPC infeasible at step', i)
        break
    state = rocket_dynamics(state, control_action)
    state_trajectory.append(state)
    action_trajectory.append(control_action)

# Convert trajectories for plotting
state_trajectory = np.array(state_trajectory)
action_trajectory = np.array(action_trajectory)

# Plotting
plt.figure(figsize=(10, 6))
plt.subplot(3, 1, 1)
plt.plot(state_trajectory[:, 0])
plt.ylabel('Height (m)')
plt.title('Rocket Height')

plt.subplot(3, 1, 2)
plt.plot(state_trajectory[:, 1])
plt.ylabel('Velocity (m/s)')
plt.title('Rocket Velocity')

plt.subplot(3, 1, 3)
plt.plot(action_trajectory)
plt.ylabel('Thrust (N)')
plt.xlabel('Time Steps')
plt.title('Control Input (Thrust)')

plt.tight_layout()
plt.show()
