"""Cart-pole (inverted pendulum on a cart) nonlinear model.

State x = [p, p_dot, theta, theta_dot]; theta = 0 is upright. Input u = cart force [N].
"""
import numpy as np
from scipy.linalg import expm

M, m, l, g = 1.0, 0.1, 0.5, 9.81  # cart mass, pole mass, pole half-length, gravity
U_MAX = 20.0                      # actuator saturation [N]
DT = 0.02                         # control period [s]


def f(x, u):
    """Continuous-time nonlinear dynamics x_dot = f(x, u)."""
    _, pd, th, thd = x
    s, c = np.sin(th), np.cos(th)
    tmp = (u + m * l * thd**2 * s) / (M + m)
    thdd = (g * s - c * tmp) / (l * (4.0 / 3.0 - m * c**2 / (M + m)))
    pdd = tmp - m * l * thdd * c / (M + m)
    return np.array([pd, pdd, thd, thdd])


def step(x, u, dt=DT, substeps=4):
    """RK4 integration with zero-order-hold input."""
    h = dt / substeps
    for _ in range(substeps):
        k1 = f(x, u)
        k2 = f(x + h / 2 * k1, u)
        k3 = f(x + h / 2 * k2, u)
        k4 = f(x + h * k3, u)
        x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return x


def linearize(eps=1e-6):
    """Jacobians of f at the upright equilibrium (continuous time)."""
    x0, u0 = np.zeros(4), 0.0
    A = np.column_stack([(f(x0 + eps * e, u0) - f(x0 - eps * e, u0)) / (2 * eps) for e in np.eye(4)])
    B = ((f(x0, u0 + eps) - f(x0, u0 - eps)) / (2 * eps)).reshape(4, 1)
    return A, B


def discretize(A, B, dt=DT):
    """Exact ZOH discretization."""
    n = A.shape[0]
    Md = expm(np.block([[A, B], [np.zeros((1, n + 1))]]) * dt)
    return Md[:n, :n], Md[:n, n:]


def simulate(policy, x0, T=6.0, dt=DT, disturbance=None):
    """Closed-loop rollout on the nonlinear plant. policy(x) -> u (before saturation).
    disturbance(t) -> extra force [N] added at the cart (not seen by the controller)."""
    N = int(round(T / dt))
    X = np.zeros((N + 1, 4)); U = np.zeros(N)
    X[0] = x0
    for k in range(N):
        u = float(np.clip(policy(X[k]), -U_MAX, U_MAX))
        d = disturbance(k * dt) if disturbance else 0.0
        U[k] = u
        X[k + 1] = step(X[k], u + d, dt)
    return np.arange(N + 1) * dt, X, U
