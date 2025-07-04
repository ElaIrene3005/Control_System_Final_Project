import numpy as np
from scipy.linalg import solve_continuous_are, eigvals
from scipy.signal import place_poles
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Define system parameters
m1, m2, m3 = 0.15, 0.5, 0.35
l1, l2, l3 = 0.15, 0.5, 0.35
g = 9.81

# Inertia matrix A
I1 = (1/3) * m1 * l1**2
I2 = (1/3) * m2 * l2**2
I3 = (1/3) * m3 * l3**2

A = np.diag([I1, I2, I3])

# Gravity matrix C
C = np.array([
    m1 * g * l1,
    m2 * g * l2,
    m3 * g * l3
])

# Input matrix B (assuming simple form)
B = np.array([
    [0],
    [0],
    [1]
])

# Output matrix D
D = np.zeros((3, 1))

# Define the state-space model
def state_space_model(t, x, A, B, u):
    return A @ x + B.flatten() * u

# Constructing state-space representation
A_ss = np.block([
    [np.zeros((3, 3)), np.eye(3)],
    [-np.linalg.inv(A) @ np.diag(C), -np.linalg.inv(A) @ np.zeros((3, 3))]
])

B_ss = np.vstack([np.zeros((3, 1)), np.linalg.inv(A) @ B])

C_ss = np.eye(6)
D_ss = np.zeros((6, 1))

# Check controllability
controllability_matrix = np.hstack([B_ss, A_ss @ B_ss, A_ss @ A_ss @ B_ss, A_ss @ A_ss @ A_ss @ B_ss, A_ss @ A_ss @ A_ss @ A_ss @ B_ss, A_ss @ A_ss @ A_ss @ A_ss @ A_ss @ B_ss])
rank_controllability = np.linalg.matrix_rank(controllability_matrix)
is_controllable = rank_controllability == A_ss.shape[0]

# Pole placement design
desired_poles = np.array([-2, -3, -4, -5, -6, -7])
K = place_poles(A_ss, B_ss, desired_poles).gain_matrix

# New state-space model with state feedback
A_cl = A_ss - B_ss @ K

# Initial conditions
x0 = np.array([1, 0, 0, 0, 0, 0])

# Time span for the simulation
t_span = (0, 10)
t_eval = np.linspace(t_span[0], t_span[1], 500)

# Simulate the closed-loop system
def closed_loop_dynamics(t, x):
    u = 0  # For simplicity, we assume zero reference input
    return state_space_model(t, x, A_cl, B_ss, u)

sol = solve_ivp(closed_loop_dynamics, t_span, x0, t_eval=t_eval)

# Plotting the results
plt.figure(figsize=(12, 8))
for i in range(6):
    plt.plot(sol.t, sol.y[i], label=f'State x{i+1}')

plt.title('State Responses with Pole-Placement Control')
plt.xlabel('Time (s)')
plt.ylabel('State values')
plt.legend()
plt.grid()
plt.show()

# Stability analysis
eigenvalues = eigvals(A_cl)

# Output results for analysis
print("Controllability Matrix:\n", controllability_matrix)
print("Rank of Controllability Matrix:", rank_controllability)
print("Is the system controllable?:", is_controllable)
print("Gain Matrix K:\n", K)
print("Closed-loop System A_cl:\n", A_cl)
print("Eigenvalues of A_cl:", eigenvalues)
print("Simulation Results (States over Time):\n", sol.y)

# Output results for further use if needed
results = {
    "controllability_matrix": controllability_matrix,
    "rank_controllability": rank_controllability,
    "is_controllable": is_controllable,
    "K": K,
    "A_cl": A_cl,
    "eigenvalues": eigenvalues,
    "simulation_results": sol.y
}
