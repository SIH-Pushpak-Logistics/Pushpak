#!/usr/bin/env python3
import numpy as np
import scipy.linalg
import yaml
import os

def calculate_lqr_gains():
    # --- Physical System Parameters ---
    g = 9.81      # Gravity (m/s^2)
    L = 0.25      # Tether length (meters)

    print(f"--- LQR Tuning Initialized ---")
    print(f"Physical Parameters: L={L}m, g={g}m/s^2")

    # --- System Matrices (State: [x, x_dot, theta, theta_dot]) ---
    # The A matrix (Dynamics)
    A = np.array([
        [0, 1,  0,    0],
        [0, 0,  0,    0],
        [0, 0,  0,    1],
        [0, 0, -g/L,  0]
    ])

    # The B matrix (Control Input - Acceleration)
    B = np.array([
        [0],
        [1],
        [0],
        [-1/L]
    ])

    # --- Tuning Matrices (The Physics Budget) ---
    # Q = diag([q_x, q_x_dot, q_theta, q_theta_dot])
    # q_x = 0 because we are a Velocity Damper, not a position-hold node.
    Q = np.diag([1e-6, 10.0, 100.0, 10.0])
    
    # R = Control effort penalty (Massive Tarot 650 frame requires smooth, non-violent thrust)
    R = np.array([[50.0]])

    # --- Solve the Continuous Algebraic Riccati Equation (CARE) ---
    print("\nSolving Riccati Equation...")
    P = scipy.linalg.solve_continuous_are(A, B, Q, R)

    # --- Compute the Optimal Gain Matrix K ---
    # K = R^-1 * B^T * P
    K = np.linalg.inv(R).dot(B.T).dot(P)
    
    print("\nOptimal Gain Matrix K:")
    print(np.round(K, 4))

    # --- Extract Specific Axis Gains ---
    # Because a pendulum is physically symmetrical on the X and Y axes,
    # the exact same gains apply to both the roll/sway and pitch/surge dynamics.
    # K matrix structure: [k_x, k_x_dot, k_theta, k_theta_dot]
    k_theta = float(K[0, 2])
    k_theta_dot = float(K[0, 3])

    print(f"\nExtracted Gains:")
    print(f"k_theta: {k_theta:.4f}")
    print(f"k_theta_dot: {k_theta_dot:.4f}")

    # --- Generate ROS 2 Parameter Configuration ---
    yaml_data = {
        'anti_sway_filter_node': {
            'ros__parameters': {
                'k_theta_x': k_theta,
                'k_theta_dot_x': k_theta_dot,
                'k_theta_y': k_theta,
                'k_theta_dot_y': k_theta_dot
            }
        }
    }

    # Save to the config directory
    output_path = "sway_params.yaml"
    with open(output_path, 'w') as outfile:
        yaml.dump(yaml_data, outfile, default_flow_style=False)
    
    print(f"\nSUCCESS: LQR configuration saved to {output_path}")

if __name__ == '__main__':
    calculate_lqr_gains()