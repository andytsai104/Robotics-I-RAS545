import numpy as np
from scipy.optimize import minimize

# Desired end-effector position
x = 372.113
y =  -178.3
z = 70
desired_position = np.array([x, y, z])

def forward_kinematics(q):
    th1, th2, th3, th4, th5, th6 = q
        
    # Position components (simplified notation)
    Px = (
    250 * np.cos(th1) * np.sin(th2)
    - (1091 * np.sin(th1)) / 10
    - (3793 * np.cos(th5) * np.sin(th1)) / 50
    + 250 * np.cos(th1) * np.cos(th2) * np.sin(th3)
    + 250 * np.cos(th1) * np.cos(th3) * np.sin(th2)
    + 108 * np.cos(th1) * np.cos(th2) * np.cos(th3) * np.sin(th4)
    + 108 * np.cos(th1) * np.cos(th2) * np.cos(th4) * np.sin(th3)
    + 108 * np.cos(th1) * np.cos(th3) * np.cos(th4) * np.sin(th2)
    - 108 * np.cos(th1) * np.sin(th2) * np.sin(th3) * np.sin(th4)
    - (3793 * np.cos(th1) * np.cos(th2) * np.cos(th3) * np.cos(th4) * np.sin(th5)) / 50
    + (3793 * np.cos(th1) * np.cos(th2) * np.sin(th3) * np.sin(th4) * np.sin(th5)) / 50
    + (3793 * np.cos(th1) * np.cos(th3) * np.sin(th2) * np.sin(th4) * np.sin(th5)) / 50
    + (3793 * np.cos(th1) * np.cos(th4) * np.sin(th2) * np.sin(th3) * np.sin(th5)) / 50
    )

    Py = (
        (1091 * np.cos(th1)) / 10
        + (3793 * np.cos(th1) * np.cos(th5)) / 50
        + 250 * np.sin(th1) * np.sin(th2)
        + 250 * np.cos(th2) * np.sin(th1) * np.sin(th3)
        + 250 * np.cos(th3) * np.sin(th1) * np.sin(th2)
        + 108 * np.cos(th2) * np.cos(th3) * np.sin(th1) * np.sin(th4)
        + 108 * np.cos(th2) * np.cos(th4) * np.sin(th1) * np.sin(th3)
        + 108 * np.cos(th3) * np.cos(th4) * np.sin(th1) * np.sin(th2)
        - 108 * np.sin(th1) * np.sin(th2) * np.sin(th3) * np.sin(th4)
        - (3793 * np.cos(th2) * np.cos(th3) * np.cos(th4) * np.sin(th1) * np.sin(th5)) / 50
        + (3793 * np.cos(th2) * np.sin(th1) * np.sin(th3) * np.sin(th4) * np.sin(th5)) / 50
        + (3793 * np.cos(th3) * np.sin(th1) * np.sin(th2) * np.sin(th4) * np.sin(th5)) / 50
        + (3793 * np.cos(th4) * np.sin(th1) * np.sin(th2) * np.sin(th3) * np.sin(th5)) / 50
    )

    Pz = (
        108 * np.cos(th2 + th3 + th4)
        - (3793 * np.cos(th2 + th3 + th4 + th5)) / 100
        + 250 * np.cos(th2 + th3)
        + 250 * np.cos(th2)
        + (3793 * np.cos(th2 + th3 + th4 - th5)) / 100
        + 10967 / 50
    )

    return np.array([Px, Py, Pz])

def error_function(q):
    current_position = forward_kinematics(q)
    return np.sum((current_position - desired_position)**2)  # Squared error

# Better initial guess - middle of possible range
initial_guess = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # All zeros often works well

# Realistic joint limits (in radians)
bounds = [
    (-np.pi, np.pi),    # q1
    (-np.pi/2, np.pi/2), # q2 
    (-np.pi/2, np.pi/2), # q3
    (-np.pi/2, np.pi/2), # q4
    (-np.pi/2, np.pi/2), # q5
    (-np.pi, np.pi)      # q6
]

# Optimization options
options = {
    'maxiter': 1000,
    'ftol': 1e-6,
    'eps': 1e-8
}

# Try multiple optimization methods
for method in ['L-BFGS-B', 'SLSQP', 'TNC']:
    print(f"\nTrying method: {method}")
    result = minimize(
        error_function,
        initial_guess,
        method=method,
        bounds=bounds,
        options=options
    )
    
    if result.success:
        optimized_angles = result.x
        print("Success!")
        print("Joint angles (rad):", optimized_angles)
        print("Joint angles (deg):", np.degrees(optimized_angles))
        print("Achieved position:", forward_kinematics(optimized_angles))
        print("Error:", np.linalg.norm(forward_kinematics(optimized_angles) - desired_position))
        break
    else:
        print(f"Method {method} failed:", result.message)
else:
    print("All optimization methods failed. Trying with relaxed tolerances...")
    result = minimize(
        error_function,
        initial_guess,
        method='L-BFGS-B',
        bounds=bounds,
        options={'ftol': 1e-3, 'maxiter': 5000}
    )
    if result.success:
        optimized_angles = result.x
        print("Success with relaxed tolerances:")
        print("Joint angles (rad):", optimized_angles)
    else:
        print("Final attempt failed. Consider:")
        print("1. Different initial guess")
        print("2. Checking forward kinematics implementation")
        print("3. Adding orientation constraints")