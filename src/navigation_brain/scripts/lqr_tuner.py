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
    A = np.array([
        [0, 1,  0,    0],
        [0, 0,  0,    0],
        [0, 0,  0,    1],
        [0, 0, -g/L,  0]
    ])

    B = np.array([
        [0],
        [1],
        [0],
        [-1/L]
    ])

    # --- Tuning Matrices (The Physics Budget) ---
    # Q = diag([q_x, q_x_dot, q_theta, q_theta_dot])
    # q_x_dot is penalized at 10.0 to prevent runaway velocity.
    Q = np.diag([1e-6, 10.0, 100.0, 10.0])
    
    # R = Control effort penalty 
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
    # K matrix structure: [k_x, k_v, k_theta, k_theta_dot]
    # We ignore k_x (index 0) because this is a velocity damper, not position hold.
    k_v = float(K[0, 1])
    k_theta = float(K[0, 2])
    k_theta_dot = float(K[0, 3])

    print(f"\nExtracted Gains:")
    print(f"k_v (Velocity Penalty): {k_v:.4f}")
    print(f"k_theta (Swing Penalty): {k_theta:.4f}")
    print(f"k_theta_dot (Swing Rate Penalty): {k_theta_dot:.4f}")

    # --- Generate ROS 2 Parameter Configuration ---
    # Pushing the FULL state feedback pipeline to the node
    yaml_data = {
        'anti_sway_filter_node': {
            'ros__parameters': {
                'k_v_x': k_v,
                'k_theta_x': k_theta,
                'k_theta_dot_x': k_theta_dot,
                'k_v_y': k_v,
                'k_theta_y': k_theta,
                'k_theta_dot_y': k_theta_dot
            }
        }
    }

    output_path = "sway_params.yaml"
    with open(output_path, 'w') as outfile:
        yaml.dump(yaml_data, outfile, default_flow_style=False)
    
    print(f"\nSUCCESS: LQR configuration saved to {output_path}")

if __name__ == '__main__':
    calculate_lqr_gains()