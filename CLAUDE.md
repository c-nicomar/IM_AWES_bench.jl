# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

Julia package for reproducing an induction-machine (IM) AWES ground-station bench simulation (originally a Simulink model). It provides two modelling paths:

1. **Hybrid FOC path** (current focus, the package core) — a hand-written induction-machine plant stepped alongside hand-written discrete controllers/observers on a fixed-step loop, closer to a real embedded implementation. Plain Julia over concrete `Float64`, no symbolic types.
2. **Scalar V/f and symbolic FOC path** — ModelingToolkit (MTK) systems for basic plant validation, living in a package extension.

Documentation:
- [docs/src/architecture.md](docs/src/architecture.md) — architecture, sign conventions, full description of controllers/estimators/simulators. Read before making non-trivial changes to any of them.
- [docs/src/overview.md](docs/src/overview.md) — purpose, supported workflows, use as machine-control backend for `ElectricMachineWinch.jl`.
- [docs/src/mtpa.md](docs/src/mtpa.md) — MTPA speed controller and its include-order constraint.
- [docs/src/foc_f1_160kw.md](docs/src/foc_f1_160kw.md) — 160 kW machine case: bipolar speed tracking, field weakening, voltage/current limits.
- [oldplans/](oldplans/) — completed refactoring plans (package extension, Makie migration). Historical, but they record *why* the current structure looks the way it does.

## Commands

Julia **1.12 is required** (`bin/install` enforces it). This is a Julia workspace: `Project.toml` `[workspace] projects = ["scripts", "test"]`, each with its own `Project.toml` sourcing `InductionMachineDrives` from `..` via `[sources]`.

```bash
bin/install              # check Julia 1.12, restore Manifest-v1.12.toml from the
                         # .default, drop stale sub-project Manifest.toml files,
                         # instantiate the workspace, precompile scripts/
bin/run_julia            # REPL: julia -t auto --project, KMP_DUPLICATE_LIB_OK set,
                         # custom sysimage if present; defines menu() / menu2()
bin/run_julia --nosysimage   # same, but ignore bin/sysimage.*
bin/create_sys_image     # build bin/sysimage.* (20-40 min) via test/create_sys_image.jl
```

In the REPL started by `bin/run_julia`, `menu()` lists the hybrid FOC examples and `menu2()` the ModelingToolkit ones (see [scripts/menu.jl](scripts/menu.jl), [scripts/menu2.jl](scripts/menu2.jl)). The split is deliberate: only `menu2()` scripts pull in MTK/OrdinaryDiffEq.

Run the test suite:
```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```
or, in a live REPL, `include("test/runtests.jl")`. It is slow by design — four of the testsets spawn fresh `julia` subprocesses to check extension loading (that is the only way to test "absent before, present after"), and the MTK testsets pay for `mtkcompile` plus extension precompilation. `test/runtests.jl` prints timestamped progress lines so a healthy run is distinguishable from a hang.

Run a single simulation script (each activates `scripts/Project.toml` itself):
```bash
julia scripts/run_foc_current_hybrid_steps.jl
```
Hybrid FOC scripts: `run_foc_current_hybrid_steps.jl`, `run_foc_torque_f1_steps.jl`, `run_foc_speed_f1_ramp_load_steps.jl`, `run_foc_speed_f1_ramp_load_estimator.jl`, `run_foc_speed_f1_awes_profile.jl`, `run_foc_speed_f1_awes_profile_efficiency.jl`, `run_foc_speed_mtpa_awes_profile.jl`, `run_foc_speed_f1_160kw.jl`.

MTK scripts: `run_scalar_im.jl`, `run_scalar_frequency_steps.jl`, `run_scalar_frequency_steps_load_steps.jl`, `run_foc_current_steps.jl`.

Scripts write CSV output to `results/` (gitignored) and plot with `MakieControlPlots`. `bin/run_julia` and `bin/install` export `KMP_DUPLICATE_LIB_OK=TRUE`; set it yourself if you start Julia any other way, especially on Windows.

`Manifest-v1.12.toml` is gitignored but `Manifest-v1.12.toml.default` is committed — that is the known-good resolution `bin/install` restores. A `Manifest.toml` inside `scripts/` or `test/` shadows the shared workspace manifest and breaks resolution; `bin/install` deletes them.

## Architecture

### Zero-dependency core plus MTK extension
The package `[deps]` is **empty**. `ModelingToolkit` and `OrdinaryDiffEq` are `[weakdeps]` triggering the `InductionMachineDrivesMTKExt` extension in `ext/`. Everything symbolic lives there; `src/` is MTK-free.

```julia
using InductionMachineDrives                                    # hybrid simulators only
using ModelingToolkit, OrdinaryDiffEq, InductionMachineDrives   # + symbolic system builders
```

Both triggers are required — Julia loads an extension only once *every* package in the trigger list is present, and `OrdinaryDiffEq` is not optional because the `simulate_*` functions default to `Rodas5P()` and call `solve`. `using ModelingToolkit` alone leaves the builders at zero methods, which is the intended signal; a `MethodError` listing no methods means a trigger is missing, not that the function is broken.

Consequences to respect when changing things:
- An extension **cannot create a binding in its parent**. `build_scalar_im_system`, `simulate_scalar_im`, `build_foc_current_im_system`, `simulate_foc_current_im` are declared as empty stubs in [src/InductionMachineDrives.jl](src/InductionMachineDrives.jl) and only *get methods* from the extension. Same for the `build_scalar_im_model` backward-compatible alias, which is a `const` in the main module.
- Do not add a hard dependency to the package `Project.toml` to share code. `CSV`/`DataFrames` belong to `scripts/`; shared script code belongs in `scripts/`, not `src/`.
- Equation builders in `ext/` write `D(x) ~ ...` relying on the `D_nounits as D` import at the top of `ext/InductionMachineDrivesMTKExt.jl`. Moving a builder out of that module without carrying the import fails at parse time.
- `test/runtests.jl` pins this contract; if those testsets fail, the package has quietly reacquired a hard MTK dependency.

### Module layout

`src/` — MTK-free hybrid path, all included in fixed order by [src/InductionMachineDrives.jl](src/InductionMachineDrives.jl):
- `src/controls/FOC/` — discrete inner current controller plus three outer-loop variants: F1 constant-flux torque, F1 constant-flux speed, MTPA speed with field weakening.
- `src/estimators/` — discrete rotor-flux/torque observer and a load-torque Kalman estimator.
- `src/simulators/` — hybrid simulators stepping the hand-written plant derivative alongside the discrete blocks at a fixed sample time (current, torque-F1, speed-F1, speed-MTPA, speed-F1-160kW).

`ext/` — symbolic path, included by `ext/InductionMachineDrivesMTKExt.jl`:
- `ext/profiles/` — frequency and load reference profile builders.
- `ext/controls/` — `scalar_vf_control.jl` (V/f) and `FOC/current_controller.jl`.
- `ext/plants/induction_machine_alpha_beta.jl` — symbolic IM model in stationary alpha-beta coordinates.
- `ext/systems/` — assembles the scalar and FOC-current MTK systems from profiles + controls + plant.

**Include order matters**: `src/simulators/hybrid_foc_speed_f1_simulator.jl` must come before `hybrid_foc_speed_mtpa_simulator.jl`, because `interp_profile_linear` and the other CSV profile-playback helpers are defined in the F1 simulator and reused by the MTPA one.

Each discrete controller/estimator follows the same convention: a `*State`, `*Params`, `*Output` struct trio plus a `*_step!(...)` mutating function (e.g. `CurrentControllerDiscreteState/Params/Output` + `current_controller_step!`).

### Hybrid FOC simulation loop
Each fixed-step iteration: read plant currents/speed/angle → update rotor-flux/torque observer → optionally update load-torque estimator → update outer loop (torque or speed) if present → update inner current controller → convert dq voltage command to alpha-beta → integrate plant one sample period → store results.

### Critical sign convention
The mechanical equation used throughout plant, estimator, and feedforward code is:
```
J * dω/dt = Te + TL - B * ω
```
i.e. both electromagnetic torque `Te` and external/load torque `TL` are positive when accelerating toward positive speed; friction `B*ω` opposes motion. Getting this backwards silently breaks load feedforward and Kalman estimator sign conventions — when adding or modifying feedforward/estimator code, cross-check against [docs/src/architecture.md](docs/src/architecture.md) sections 3, 9, and 11, which spell out the exact required signs (`load_ff_sign = -1.0`, Kalman `A12 = Ts/J` not `-Ts/J`, `TL_kalman_limit_positive = false` for AWES profiles with signed torque).

### AWES CSV profiles
Input profiles (e.g. `profiles/delta_kite_13_ms_profiles.csv`) have columns `t_s, speed_ref_rpm, speed_ref_rad_s, torque_ref_Nm` and are linearly interpolated at each simulation timestep (Simulink-timeseries-like behavior).

## System image

`bin/create_sys_image` runs [test/create_sys_image.jl](test/create_sys_image.jl), which bakes `OrdinaryDiffEq`, `MakieControlPlots` into `bin/sysimage.*` (gitignored) using the `scripts/` environment plus a representative precompile workload. `InductionMachineDrives` itself is deliberately **not** baked in, so it stays editable under Revise without a rebuild. `ModelingToolkit`, `CSV` and `DataFrames` are not baked in either; `bin/create_sys_image` precompiles them against the finished image instead. Rebuild after dependency version changes; delete the file (or pass `--nosysimage`) to fall back to the stock image.
