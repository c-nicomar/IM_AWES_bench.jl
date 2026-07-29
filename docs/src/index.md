# IM\_AWES\_bench.jl

Julia package for reproducing an induction-machine (IM) AWES ground-station bench
simulation, originally built in Simulink. It provides reusable machine models,
observers, discrete controllers, estimators, and standalone benchmark simulators
for a squirrel-cage induction machine under field-oriented control (FOC).

It is also the machine-control backend for `ElectricMachineWinch.jl`, which
embeds the induction-machine model in a `KiteSimulators.jl` autopilot simulation.

## Two modelling paths

The package deliberately keeps two independent paths side by side.

**Hybrid FOC** — the package core, and where the current work happens. A
hand-written induction-machine plant is stepped alongside hand-written discrete
controllers and observers on a fixed-step loop, which is close to how a real
embedded implementation behaves. It is plain Julia over concrete `Float64`, with
no symbolic types anywhere.

**Scalar V/f and symbolic FOC** — ModelingToolkit systems used mainly for plant
validation. Everything symbolic lives in a package extension, not in `src/`.

Each fixed-step iteration of the hybrid loop does:

```text
read plant currents / speed / angle
  -> update rotor-flux and torque observer
  -> optionally update load-torque Kalman estimator
  -> update outer loop (torque or speed), if present
  -> update inner current controller
  -> convert the dq voltage command to alpha-beta
  -> integrate the plant one sample period
  -> store results
```

## Mechanical sign convention

This one convention runs through the plant, the estimators, and every
feedforward path:

```text
J * dω/dt = Te + TL - B * ω
```

- `ω` — mechanical speed of the machine, rad/s
- `Te` — electromagnetic torque, Nm
- `TL` — external or load torque, Nm
- `B * ω` — viscous friction, opposing motion

Both `Te` and `TL` are **positive when they accelerate the shaft toward positive
speed**; a positive `TL` pulls the shaft in the positive direction rather than
resisting it.

!!! warning "Getting this backwards fails silently"

    A flipped sign here does not throw — it quietly breaks load feedforward and
    the Kalman estimator. Before changing feedforward or estimator code,
    cross-check sections 3, 9 and 11 of [the architecture page](architecture.md),
    which spell out the required signs (`load_ff_sign = -1.0`, Kalman
    `A12 = Ts/J` rather than `-Ts/J`, and `TL_kalman_limit_positive = false` for
    AWES profiles carrying signed torque).

## Loading the package

Julia **1.12 is required**. The two MTK-backed model builders are gated behind a
package extension:

```julia
using IM_AWES_bench                                    # hybrid FOC simulators only
using ModelingToolkit, OrdinaryDiffEq, IM_AWES_bench   # + the symbolic system builders
```

Both triggers are needed. Julia loads an extension only once *every* package in
its trigger list is present, and `OrdinaryDiffEq` is not optional — the
`simulate_*` functions default to `Rodas5P()` and call `solve`.

!!! note "A `MethodError` with no methods listed is the expected signal"

    `build_scalar_im_system`, `simulate_scalar_im`, `build_foc_current_im_system`
    and `simulate_foc_current_im` are empty stubs until the extension loads.
    Calling one after `using ModelingToolkit` alone raises a `MethodError`
    listing zero methods. That means a trigger is missing, not that the function
    is broken.

The package's own `[deps]` is empty by design, so `using IM_AWES_bench` on its
own pulls in nothing.

## Getting started

Clone the repository and set up the workspace:

```bash
bin/install       # checks Julia 1.12, restores the known-good manifest,
                  # instantiates the workspace, precompiles scripts/
bin/run_julia     # REPL with the right project and environment variables
```

In that REPL, `menu()` lists the hybrid FOC examples and `menu2()` the
ModelingToolkit ones. The split is deliberate: only the `menu2()` scripts pull in
ModelingToolkit and OrdinaryDiffEq.

A single script can also be run directly — each activates the `scripts/`
environment itself:

```bash
julia scripts/run_foc_current_hybrid_steps.jl
```

Scripts write CSV output to `results/` and plot with `MakieControlPlots`.

!!! tip "Set `KMP_DUPLICATE_LIB_OK`"

    `bin/install` and `bin/run_julia` export `KMP_DUPLICATE_LIB_OK=TRUE`. If you
    start Julia any other way, set it yourself — especially on Windows.

## AWES reference profiles

Input profiles such as `profiles/delta_kite_13_ms_profiles.csv` carry the columns
`t_s`, `speed_ref_rpm`, `speed_ref_rad_s` and `torque_ref_Nm`. They are linearly
interpolated at each simulation timestep, matching how a Simulink timeseries
source behaves.

## Contents

```@contents
Pages = ["overview.md", "architecture.md", "mtpa.md", "foc_f1_160kw.md"]
Depth = 2
```
