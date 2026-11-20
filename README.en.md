# omr — Constrained Tracking Control of a Three-Wheeled Omnidirectional Robot (reproduction of 22482)

Five folders by responsibility; `omr_setup.m` puts every sub-folder on the MATLAB path in one call.

```
omr/
├─ omr_setup.m     path initialisation (the single entry point)
├─ core/           L1 simulation core
│   ├─ omr_sim.m      fixed-step integration (ode1/ode3/ode4), returns r
│   └─ omr_rhs.m      closed-loop right-hand side: dq/dV/controller-state derivatives
├─ control/        L2 controller
│   ├─ omr_ctrl_eval.m          dispatches on p.controller ("v1" / "v2")
│   ├─ omr_control_core.m       single implementation of eqs. (6)(8)(10)-(12)(16)(18)(22)(24)(25)(34)
│   ├─ omr_alpha.m              virtual control, eq. (18) (shared by the control law and the initial condition)
│   ├─ omr_control.m            v1: Ĝ ≡ 0 (first version, no RBF network)
│   ├─ omr_control_v2.m         v2: Ĝ comes from the RBFNN observer (second version)
│   └─ omr_rbfnn_observer_v2.m  eqs. (9)+(35): Ĝ and dŴ/dt, with projection and diagnostics
├─ model/          L3 data
│   ├─ omr_params.m        presets ("v1" / "v2"), derived matrices, index struct p.idx
│   └─ omr_reference.m     reference trajectory and disturbance
├─ view/           visualisation
│   ├─ omr_animate.m          builds the three-wheeled-robot animation GIF from r
│   └─ omr_animate_compare.m  overlay animation of the two versions (trails / error cursor / zoom on the reference point)
├─ app/            entry points
│   ├─ run_demo.m              one-shot reproduction of the first version (curves + metrics + animation)
│   ├─ run_compare_observer.m  first vs second version (table + four-panel figure)
│   ├─ testbed_run.m           multi-scenario testbed (drives model_OMR_extend_R2022b.slx)
│   └─ testbed_scenarios.m     scenario definitions
└─ out/            run artefacts
```

## The two versions

The versions differ in **exactly one point**: where the estimate `Ĝ` of the unknown term
`G = A1·V + F(V)` in eq. (25) comes from. Every other equation, gain, regularisation term and
plant setting is shared through `omr_control_core.m`.

| Version | Entry point | Source of Ĝ |
|---|---|---|
| `v1` (first) | `omr_control.m` | `Ĝ ≡ 0` — the unknown term is not compensated at all |
| `v2` (second) | `omr_control_v2.m` → `omr_rbfnn_observer_v2.m` | RBF network: `Ĝ_i = Ŵ_iᵀφ_i`, weights adapted by eq. (35) |

## How to run

```matlab
addpath('<...>/omr');  omr_setup();
run_demo(80, 1e-3)               % first version: figures + metrics + animation
run_compare_observer(80, 1e-3)   % both versions: table + fig_v1_vs_v2.png
testbed_run()                    % drives the Simulink model through three scenarios
                                 % (needs R2022b and model_OMR_extend_R2022b.slx)
```

Each entry script locates the `omr/` root and calls `omr_setup()` itself, so putting only
`omr/app` on the path is enough to run any of them.

## 80 s results (ode4, 1 ms, q0 = [0.4; -0.4; 0.3])

The run is split into two phases: **0–40 s seeking** (the robot drives from q0 onto the
reference curve, errors of the order of decimetres) and **40–80 s pure tracking** (already on
the curve, errors of the order of centimetres). The peak metrics below are evaluated **only on
the tracking phase** — a full-run peak is dominated by the initial approach and says nothing
about control quality.

| Metric | v1 (first) | v2 (RBF) | Change |
|---|---|---|---|
| rms\|e1\| (whole run) | 0.0651 m | 0.0613 m | −5.9 % |
| **rms\|e1\| (tracking)** | 0.0297 m | 0.0124 m | **−58.1 %** |
| **max\|e1\| (tracking)** | 0.0559 m | 0.0243 m | **−56.6 %** |
| final \|e1\| | 0.0536 m | 0.0171 m | −68.1 % |
| max\|e2\| (tracking) | 2.748 | 1.089 | −60.4 % |
| max\|τ\| (tracking) | 6.58 N·m | 14.78 N·m | +124.8 % (worse) |
| max\|e1\| (seeking, for reference) | 0.4316 m | 0.4031 m | — |
| rms\|G−Ĝ\| | 0.0342 | 0.0472 | — |

`max|e1|` stays below the 0.5 m barrier in both versions (an order of magnitude below it during
tracking); `max|e2|` exceeds the instantaneous 0.5 bound at the samples where λ2 passes near
zero, but both runs converge. The identification error of v2 is actually *larger* than `|G|`
itself (0.047 vs 0.034): what the RBF network buys is a phase-correct partial compensation, not
accuracy — it cuts the tracking error by more than half.

**Read the torque peak separately.** During pure tracking, `max|τ|` of v2 is 2.2× *larger* than
that of v1: the update law (35) is driven directly by λ2, so the thin-barrier spikes of λ2 turn
into high-frequency torque dither — that is exactly why the smoothness gain is limited (analysis
in section 5 of the PDF report). The full-run "25.5 → 14.8 N·m" is an artefact of the approach
transient: v1's 25.5 N·m peak happens during 0–40 s.

## Parameters and fixed settings

| Item | Value | Notes |
|---|---|---|
| `k1 / k2` | 0.5 / 0.1 | barrier feedback gains (values from the paper) |
| `b1 = b2` | 0.5 | error barrier bounds |
| `τ_max` | ±300 N·m | actuator limit; the plant receives `sat(τ)` (eq. 8) |
| `lam2_floor` | `1e-3` | `|λ2|` can drop to 1e-8; dividing directly yields 1e4 N·m |
| `ratio_max` | 50 | cap on `λ1/λ2`, same reason |
| `lam2_max` | 75.9 | derived from `τ_max/(1.5·‖Binv‖∞)` |
| `ξ1(0)` | computed as `α(0)` | changing any parameter never leaves a stale hard-coded value |
| RBF nodes | 11 per axis, σ = 2, ρ = ρ_w = 2 | eqs. (9)/(35), plus a `‖W‖ ≤ 50` projection |

## Relation to the paper and to the Simulink model

- Row 2 of `B` in eq. (5) is printed as `+√3b1, +√3b1`, which makes `B` singular; eq. (3) on the
  same page writes `(√3/2)(f1 − f2)` with its own minus sign. The code uses
  `[√3b1, −√3b1, 0]` (as eq. (3) requires) and asserts `det(B)` at start-up.
- The denominator of eq. (18) is `2πe1` in the paper (a removable singularity at `e1 → 0`);
  the delivered `.slx` writes `2πe1 + 0.01`, which turns it into a genuine pole at
  `e1 = −1.592e-3`. The code keeps the `+ 0.01` of the delivered model, which is why the
  long-horizon curves remain sensitive to the sampling instants.
- The delivered `.slx` writes the `λ1/λ2` term as `lambda1/lambda2.*S*e2`, i.e. a matrix right
  division (a rank-1 3×3 matrix that only uses the `argmax|λ2|` channel). This repository follows
  the paper's element-wise reading and no longer reproduces that Simulink artefact.

## Dependencies

`app → omr_sim → {omr_ctrl_eval → omr_control_core, omr_rhs} → omr_reference`;
the `p` produced by `omr_params` is read-only for every module; `omr_animate` only consumes the
output `r` of `omr_sim`.

## `out/` artefacts

| File | Content |
|---|---|
| `fig_v1_tracking.png` / `fig_v1_control.png` | first version: trajectory/error and control figures (shaded = tracking phase) |
| `fig_v1_vs_v2.png` | four-panel comparison (‖e1‖, max\|τ\|, G vs Ĝ, metric bars) |
| `metrics_v1.csv` / `metrics_v2.csv` / `compare_v1_v2.csv` | metric tables (columns split all-run / tracking-phase / seeking-phase) |
| `run_v1.mat` / `run_v2.mat` | simulation logs, **decimated by 10** (take peaks from the CSV) |
| `omr_animation.gif` | first version: three-wheeled-robot animation, 161 frames at 20 fps (8.1 s) |
| `omr_compare.gif` | **overlay animation of both versions**: trails, ‖e1‖(t) cursor, and a zoom on the reference point showing both robots at the same instant |
