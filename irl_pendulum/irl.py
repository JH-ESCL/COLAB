"""Inverse reinforcement learning for linear-quadratic control.

1) System identification of (A, B) from demonstration transitions (least squares).
2) MaxEnt IRL: expert policy pi(u|x) ∝ exp(-Q_soft(x,u)) is Gaussian N(-Kx, (R+B'PB)^-1)
   for an LQ reward, so the cost weights (Q, R) are found by maximizing the demo
   log-likelihood; the noise level makes the absolute cost scale identifiable.
3) Inverse-DARE (inverse optimal control): from a least-squares estimate of the expert gain,
   solve the linear equations of the Riccati optimality conditions for P and diag(Q) >= 0
   with R fixed (scale is not identifiable from K alone).
"""
import numpy as np
from scipy.optimize import minimize, lsq_linear
from lqr import dlqr


def stack(demos):
    X = np.vstack([d['X'][:-1] for d in demos])
    Xn = np.vstack([d['X'][1:] for d in demos])
    U = np.concatenate([d['U'] for d in demos]).reshape(-1, 1)
    return X, U, Xn


def identify_model(demos):
    X, U, Xn = stack(demos)
    Z = np.hstack([X, U])
    Theta, *_ = np.linalg.lstsq(Z, Xn, rcond=None)
    return Theta[:4].T, Theta[4:].T


def estimate_gain(demos):
    X, U, _ = stack(demos)
    Kt, *_ = np.linalg.lstsq(X, -U, rcond=None)
    return Kt.T


def maxent_irl(A, B, demos, theta0=None, verbose=True):
    """theta = log([q1, q2, q3, q4, r]). Returns Q, R and the optimization history."""
    X, U, _ = stack(demos)
    N = len(U)
    hist = []

    def unpack(th):
        w = np.exp(th)
        return np.diag(w[:4]), np.array([[w[4]]])

    def nll(th):
        Q, R = unpack(th)
        try:
            K, _, S = dlqr(A, B, Q, R)
        except Exception:
            return 1e10
        e = U + X @ K.T                              # u - (-Kx)
        Si = np.linalg.inv(S)
        return 0.5 * (np.sum((e @ Si) * e) / N + np.log(np.linalg.det(2 * np.pi * S)))

    def cb(th):
        hist.append(nll(th))
        if verbose and len(hist) % 10 == 0:
            print(f'  iter {len(hist):3d}  mean NLL = {hist[-1]:.5f}')

    th0 = np.zeros(5) if theta0 is None else theta0
    hist.append(nll(th0))
    res = minimize(nll, th0, method='BFGS', callback=cb, options={'gtol': 1e-7, 'maxiter': 500})
    Q, R = unpack(res.x)
    return Q, R, np.array(hist), res


def inverse_dare(A, B, K, R=np.eye(1)):
    """Find symmetric P and diagonal Q >= 0 such that
       (R + B'PB) K = B'PA                         (stationarity)
       P = (A-BK)' P (A-BK) + K'RK + Q             (Riccati / Lyapunov)
    in least squares; both are linear in (P, Q) for a known K."""
    n = A.shape[0]
    iu = np.triu_indices(n)
    npar = len(iu[0])

    def Pmat(v):
        P = np.zeros((n, n)); P[iu] = v
        return P + P.T - np.diag(np.diag(P))

    Acl = A - B @ K
    rows, rhs = [], []
    basis = [Pmat(e) for e in np.eye(npar)]
    # stationarity: B'P B K - B'P A = -R K   (1 x n equations)
    Ms = np.array([(B.T @ E @ B @ K - B.T @ E @ A).ravel() for E in basis]).T
    rows.append(np.hstack([Ms, np.zeros((n, n))])); rhs.append((-R @ K).ravel())
    # Riccati: P - Acl'P Acl - Q = K'RK   (upper triangle)
    Mr = np.array([(E - Acl.T @ E @ Acl)[iu] for E in basis]).T
    Qcols = np.array([np.diag(e)[iu] for e in np.eye(n)]).T
    rows.append(np.hstack([Mr, -Qcols])); rhs.append((K.T @ R @ K)[iu])
    Amat, b = np.vstack(rows), np.concatenate(rhs)
    scale = np.linalg.norm(Amat, axis=0); scale[scale == 0] = 1
    lb = np.r_[-np.inf * np.ones(npar), np.zeros(n)]
    sol = lsq_linear(Amat / scale, b, bounds=(lb * scale, np.inf), tol=1e-12, max_iter=5000)
    z = sol.x / scale
    return np.diag(z[npar:]), Pmat(z[:npar]), np.linalg.norm(Amat @ z - b)
