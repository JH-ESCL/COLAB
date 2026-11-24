import numpy as np
from scipy.linalg import solve_discrete_are


def dlqr(A, B, Q, R):
    """Discrete LQR: returns gain K (u = -Kx), Riccati P, and MaxEnt policy covariance."""
    P = solve_discrete_are(A, B, Q, R)
    H = R + B.T @ P @ B                      # Hessian of the Q-function w.r.t. u
    K = np.linalg.solve(H, B.T @ P @ A)
    return K, P, np.linalg.inv(H)            # Boltzmann policy: u ~ N(-Kx, H^-1)


def quad_cost(X, U, Q, R):
    """Accumulated true stage cost sum x'Qx + u'Ru."""
    return float(np.einsum('ij,jk,ik->', X[:-1], Q, X[:-1]) + R[0, 0] * np.sum(U**2))
