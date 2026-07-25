# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

Julia package for reproducing an induction-machine (IM) AWES ground-station bench simulation (originally a Simulink model). It provides two modelling paths:

1. **Scalar V/f path** — a symbolic ModelingToolkit (MTK) system for basic plant validation.
2. **Hybrid FOC path** (current focus) — a continuous MTK/hand-written induction-machine plant driven by hand-written discrete controllers/observers on a fixed-step loop, closer to a real embedded implementation.

Full architecture details live in [docs/README_IM_AWES_bench_jl.md](docs/README_IM_AWES_bench_jl.md) — read it before making non-trivial changes to controllers, estimators, or simulators. [docs/README_MTPA_INTEGRATION.md](docs/README_MTPA_INTEGRATION.md) documents the MTPA speed-controller addition and its include-order constraint.

## Commands

This is a Julia workspace (`Project.toml` `[workspace]` = `examples`, `scripts`, `test`), each with its own `Project.toml` sourcing `IM_AWES_bench` from `..`.

Start a REPL with the right environment:
```bash
bin/run_julia            # activates project root, sets KMP_DUPLICATE_LIB_OK, julia -t auto --project
```

Run the test suite:
```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Run a single simulation script (each activates its own env against `scripts/Project.toml`):
```bash
julia --project=. scripts/run_foc_current_hybrid_steps.jl
julia --project=. scripts/run_foc_torque_f1_steps.jl
julia --project=. scripts/run_foc_speed_f1_ramp_load_steps.jl
julia --project=. scripts/run_foc_speed_f1_ramp_load_estimator.jl
julia --project=. scripts/run_foc_speed_f1_awes_profile.jl
julia --project=. scripts/run_foc_speed_mtpa_awes_profile.jl
```
Scripts write CSV output to `results/` (gitignored). On Windows, `ENV["KMP_DUPLICATE_LIB_OK"] = "TRUE"` must be set before `using ControlPlots`.

## Architecture

### Module structure
`src/IM_AWES_bench.jl` is the single entry point; it `include()`s every file in a fixed order and re-exports the public API. The order matters: **`src/simulators/hybrid_foc_speed_f1_simulator.jl` must be included before `hybrid_foc_speed_mtpa_simulator.jl`**, since the MTPA simulator reuses the F1 simulator's CSV profile-interpolation helpers.

Layout, by pipeline stage:
- `src/profiles/` — frequency/load reference profiles, including CSV playback interpolation.
- `src/controls/` — `scalar_vf_control.jl` (V/f) and `controls/FOC/` (discrete inner current controller + three outer-loop variants: F1 constant-flux torque, F1 constant-flux speed, MTPA speed with field weakening).
- `src/estimators/` — discrete rotor-flux/torque observer and a load-torque Kalman estimator.
- `src/plants/` — symbolic MTK induction-machine model in stationary alpha-beta coordinates (`induction_machine_alpha_beta.jl`).
- `src/systems/` — assembles MTK systems (scalar and FOC-current) from profiles + controls + plant.
- `src/simulators/` — hybrid simulators that step a hand-written plant derivative alongside discrete controllers/estimators at a fixed sample time (current, torque-F1, speed-F1, speed-MTPA).

Each discrete controller/estimator follows the same convention: a `*State`, `*Params`, `*Output` struct trio plus a `*_step!(...)` mutating function (e.g. `CurrentControllerDiscreteState/Params/Output` + `current_controller_step!`).

### Hybrid FOC simulation loop
Each fixed-step iteration: read plant currents/speed/angle → update rotor-flux/torque observer → optionally update load-torque estimator → update outer loop (torque or speed) if present → update inner current controller → convert dq voltage command to alpha-beta → integrate plant one sample period → store results.

### Critical sign convention
The mechanical equation used throughout plant, estimator, and feedforward code is:
```
J * dω/dt = Te + TL - B * ω
```
i.e. both electromagnetic torque `Te` and external/load torque `TL` are positive when accelerating toward positive speed; friction `B*ω` opposes motion. Getting this backwards silently breaks load feedforward and Kalman estimator sign conventions — when adding or modifying feedforward/estimator code, cross-check against [docs/README_IM_AWES_bench_jl.md](docs/README_IM_AWES_bench_jl.md) sections 3, 9, and 11, which spell out the exact required signs (`load_ff_sign = -1.0`, Kalman `A12 = Ts/J` not `-Ts/J`, `TL_kalman_limit_positive = false` for AWES profiles with signed torque).

### AWES CSV profiles
Input profiles (e.g. `profiles/delta_kite_13_ms_profiles.csv`) have columns `t_s, speed_ref_rpm, speed_ref_rad_s, torque_ref_Nm` and are linearly interpolated at each simulation timestep (Simulink-timeseries-like behavior).
