"""Inverse RL controller design for the cart-pole inverted pendulum.

Pipeline
  1. A hidden "expert" (LQR with secret Q*, R*, Boltzmann-noisy) balances the nonlinear
     cart-pole and produces demonstrations (x, u) only.
  2. IRL recovers the cost from the demos:  sys-ID (A, B) -> MaxEnt IRL / inverse-DARE.
  3. A new controller is designed from the learned cost (LQR = forward RL) and
     simulated on the nonlinear plant: nominal, large-angle, and disturbance tests.
Run:  python main.py      (figures and a GIF go to ./results)
"""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import animation

import cartpole as cp
from lqr import dlqr, quad_cost
from irl import identify_model, estimate_gain, maxent_irl, inverse_dare

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')
os.makedirs(OUT, exist_ok=True)
rng = np.random.default_rng(0)

# ---------------------------------------------------------------- palette / style
INK, INK2, MUTED, GRID, AXIS, SURF = '#0b0b0b', '#52514e', '#898781', '#e1e0d9', '#c3c2b7', '#fcfcfb'
C = {'Expert (true cost)': '#2a78d6', 'MaxEnt IRL': '#eb6834', 'Inverse-DARE': '#1baf7a'}
LS = {'Expert (true cost)': '-', 'MaxEnt IRL': '--', 'Inverse-DARE': ':'}
plt.rcParams.update({
    'figure.facecolor': SURF, 'axes.facecolor': SURF, 'savefig.facecolor': SURF,
    'axes.edgecolor': AXIS, 'axes.labelcolor': INK2, 'xtick.color': MUTED, 'ytick.color': MUTED,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6,
    'axes.spines.top': False, 'axes.spines.right': False, 'axes.titlecolor': INK,
    'axes.titlesize': 11, 'axes.labelsize': 9, 'xtick.labelsize': 8, 'ytick.labelsize': 8,
    'legend.fontsize': 8, 'legend.frameon': False, 'lines.linewidth': 2, 'font.size': 9,
})

# ---------------------------------------------------------------- 1. expert
Ac, Bc = cp.linearize()
A, B = cp.discretize(Ac, Bc)
Q_true = np.diag([5.0, 0.5, 50.0, 1.0])
R_true = np.array([[0.1]])
K_true, P_true, S_true = dlqr(A, B, Q_true, R_true)
sd = np.sqrt(S_true[0, 0])
print('Expert gain K* =', np.round(K_true, 3), f'  policy noise std = {sd:.3f} N')

def demo(x0, T=5.0):
    N = int(T / cp.DT); X = np.zeros((N + 1, 4)); U = np.zeros(N); X[0] = x0
    for k in range(N):
        U[k] = (-K_true @ X[k]).item() + sd * rng.standard_normal()
        X[k + 1] = cp.step(X[k], U[k])
    return {'X': X, 'U': U}

x0s = rng.uniform([-0.5, -0.5, -0.15, -0.3], [0.5, 0.5, 0.15, 0.3], size=(20, 4))
demos = [demo(x) for x in x0s]
print(f'{len(demos)} demonstrations, {sum(len(d["U"]) for d in demos)} (x,u) samples')

# ---------------------------------------------------------------- 2. IRL
A_id, B_id = identify_model(demos)
print(f'Sys-ID relative error: A {np.linalg.norm(A_id-A)/np.linalg.norm(A):.2e}, '
      f'B {np.linalg.norm(B_id-B)/np.linalg.norm(B):.2e}')

print('MaxEnt IRL ...')
Q_me, R_me, nll_hist, res = maxent_irl(A_id, B_id, demos)
K_me, _, _ = dlqr(A, B, Q_me, R_me)

K_hat = estimate_gain(demos)
Q_id, _, resid = inverse_dare(A_id, B_id, K_hat, R=R_me)   # fix R to MaxEnt's scale for comparability
K_id, _, _ = dlqr(A, B, Q_id + 1e-9 * np.eye(4), R_me)
print(f'Inverse-DARE residual = {resid:.2e}')

def wvec(Q, R): return np.r_[np.diag(Q), R[0, 0]]
names = ['q_p', 'q_pd', 'q_th', 'q_thd', 'r']
print('\n{:>8s} {:>10s} {:>10s} {:>12s}'.format('weight', 'true', 'MaxEnt', 'Inv-DARE'))
for n, a, b, c in zip(names, wvec(Q_true, R_true), wvec(Q_me, R_me), wvec(Q_id, R_me)):
    print(f'{n:>8s} {a:10.4f} {b:10.4f} {c:12.4f}')
print('\nGains:\n K*      ', np.round(K_true, 3), '\n K_MaxEnt', np.round(K_me, 3),
      '\n K_InvDARE', np.round(K_id, 3))

ctrls = {'Expert (true cost)': K_true, 'MaxEnt IRL': K_me, 'Inverse-DARE': K_id}

# ---------------------------------------------------------------- 3. simulation tests
tests = {
    'nominal':     dict(x0=np.array([0.0, 0, 0.2, 0]), dist=None, T=5),
    'large_angle': dict(x0=np.array([0.0, 0, 0.6, 0]), dist=None, T=5),
    'disturbance': dict(x0=np.zeros(4), dist=lambda t: 15.0 if 1.0 <= t < 1.1 else 0.0, T=5),
}
sims = {}
print('\nTrue-cost J on nonlinear plant (lower is better):')
for tn, tc in tests.items():
    sims[tn] = {}
    for cn, K in ctrls.items():
        t, X, U = cp.simulate(lambda x, K=K: (-K @ x).item(), tc['x0'], T=tc['T'], disturbance=tc['dist'])
        sims[tn][cn] = (t, X, U)
    js = {cn: quad_cost(s[1], s[2], Q_true, R_true) for cn, s in sims[tn].items()}
    print(f'  {tn:12s} ' + '  '.join(f'{cn}: {j:8.2f}' for cn, j in js.items()))

# Monte-Carlo: regret of learned controllers under the true cost
mc = {cn: [] for cn in ctrls}
for x0 in rng.uniform([-1, -1, -0.4, -1], [1, 1, 0.4, 1], size=(100, 4)):
    for cn, K in ctrls.items():
        _, X, U = cp.simulate(lambda x, K=K: (-K @ x).item(), x0, T=6)
        mc[cn].append(quad_cost(X, U, Q_true, R_true))
base = np.array(mc['Expert (true cost)'])
for cn in ['MaxEnt IRL', 'Inverse-DARE']:
    print(f'  Monte-Carlo (100 runs) cost ratio {cn} / expert: mean {np.mean(np.array(mc[cn])/base):.4f}')

# ---------------------------------------------------------------- figures
# Fig 1: learned vs true weights (normalized by r, log scale)
fig, axs = plt.subplots(1, 2, figsize=(10, 3.6), gridspec_kw={'width_ratios': [3, 2]})
ax = axs[0]
lab = ['$q_p$/r', r'$q_{\dot p}$/r', r'$q_\theta$/r', r'$q_{\dot\theta}$/r']
vals = {'Expert (true cost)': np.diag(Q_true) / R_true[0, 0], 'MaxEnt IRL': np.diag(Q_me) / R_me[0, 0],
        'Inverse-DARE': np.diag(Q_id) / R_me[0, 0]}
w = 0.26
for i, (cn, v) in enumerate(vals.items()):
    xs = np.arange(4) + (i - 1) * w
    ax.bar(xs, v, width=w - 0.03, color=C[cn], label=cn, zorder=3)
ax.set_yscale('log'); ax.set_xticks(range(4)); ax.set_xticklabels(lab)
ax.set_title('Recovered cost weights (ratio to input weight)', loc='left')
ax.legend(loc='upper left'); ax.grid(axis='x', visible=False)
ax = axs[1]
ax.plot(np.arange(len(nll_hist)), nll_hist, color=C['MaxEnt IRL'])
ax.axhline(0.5 * (1 + np.log(2 * np.pi * S_true[0, 0])), color=MUTED, lw=1, ls='--')
ax.text(len(nll_hist) - 1, 0.5 * (1 + np.log(2 * np.pi * S_true[0, 0])), 'entropy of true policy',
        color=INK2, fontsize=8, ha='right', va='bottom')
ax.set_title('MaxEnt IRL: demo negative log-likelihood', loc='left')
ax.set_xlabel('BFGS iteration'); ax.set_ylabel('mean NLL per sample')
fig.tight_layout(); fig.savefig(os.path.join(OUT, 'fig1_irl_weights.png'), dpi=150); plt.close(fig)

# Fig 2-4: time responses
ylabels = ['cart position p [m]', r'pole angle $\theta$ [deg]', 'force u [N]']
titles = {'nominal': r'Nominal: $\theta_0$ = 0.2 rad (11.5°)',
          'large_angle': r'Large initial angle: $\theta_0$ = 0.6 rad (34°), |u| ≤ 20 N',
          'disturbance': 'Disturbance rejection: 15 N impulse on cart at t = 1.0 s for 0.1 s'}
for fi, (tn, sim) in enumerate(sims.items(), start=2):
    fig, axs = plt.subplots(3, 1, figsize=(8, 6.4), sharex=True)
    for cn, (t, X, U) in sim.items():
        axs[0].plot(t, X[:, 0], color=C[cn], ls=LS[cn], label=cn)
        axs[1].plot(t, np.degrees(X[:, 2]), color=C[cn], ls=LS[cn])
        axs[2].plot(t[:-1], U, color=C[cn], ls=LS[cn])
    for a, yl in zip(axs, ylabels): a.set_ylabel(yl)
    if tn == 'large_angle':
        for s in (-1, 1): axs[2].axhline(s * cp.U_MAX, color=MUTED, lw=1, ls='--')
    if tn == 'disturbance': 
        for a in axs: a.axvspan(1.0, 1.1, color=GRID, zorder=0)
    axs[0].set_title(titles[tn], loc='left'); axs[0].legend(loc='upper right', ncol=3)
    axs[-1].set_xlabel('time [s]')
    fig.tight_layout(); fig.savefig(os.path.join(OUT, f'fig{fi}_{tn}.png'), dpi=150); plt.close(fig)

# Fig 5: Monte-Carlo cost ratio
fig, ax = plt.subplots(figsize=(6, 3))
data = [np.array(mc[cn]) / base for cn in ['MaxEnt IRL', 'Inverse-DARE']]
bp = ax.boxplot(data, orientation='horizontal', widths=0.5, patch_artist=True, medianprops=dict(color=INK))
for patch, cn in zip(bp['boxes'], ['MaxEnt IRL', 'Inverse-DARE']):
    patch.set_facecolor(C[cn]); patch.set_edgecolor(C[cn])
ax.set_yticks([1, 2]); ax.set_yticklabels(['MaxEnt IRL', 'Inverse-DARE'])
ax.axvline(1.0, color=MUTED, lw=1, ls='--')
ax.set_xlabel('true-cost J / expert J  (100 random initial states)')
ax.set_title('Optimality of IRL controllers under the hidden true cost', loc='left')
fig.tight_layout(); fig.savefig(os.path.join(OUT, 'fig5_monte_carlo.png'), dpi=150); plt.close(fig)

# Animation (MaxEnt-IRL controller, large-angle test)
t, X, U = sims['large_angle']['MaxEnt IRL']
fig, ax = plt.subplots(figsize=(6, 3))
ax.set_xlim(-1.5, 1.5); ax.set_ylim(-0.3, 1.2); ax.set_aspect('equal'); ax.grid(False)
ax.axhline(0, color=AXIS, lw=1)
cart = plt.Rectangle((0, -0.1), 0.3, 0.2, color=C['Expert (true cost)'])
ax.add_patch(cart)
pole, = ax.plot([], [], color=C['MaxEnt IRL'], lw=4, solid_capstyle='round')
txt = ax.text(-1.45, 1.08, '', color=INK2, fontsize=8)
ax.set_title('MaxEnt-IRL controller, θ0 = 34°', loc='left')
def upd(k):
    p, th = X[k, 0], X[k, 2]
    cart.set_x(p - 0.15)
    pole.set_data([p, p + 2 * cp.l * np.sin(th)], [0, 2 * cp.l * np.cos(th)])
    txt.set_text(f't = {t[k]:.2f} s   θ = {np.degrees(th):6.1f}°   u = {U[min(k, len(U)-1)]:6.1f} N')
    return cart, pole, txt
anim = animation.FuncAnimation(fig, upd, frames=range(0, len(t), 2), blit=True)
anim.save(os.path.join(OUT, 'cartpole_irl.gif'), writer=animation.PillowWriter(fps=25))
plt.close(fig)
print('\nSaved figures to', OUT)
