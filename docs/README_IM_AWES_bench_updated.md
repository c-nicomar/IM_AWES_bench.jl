# IM_AWES_bench.jl

Julia package for modelling, simulating, and testing induction-machine control blocks for the AWES ground-station electrical bench.

The package contains both continuous ModelingToolkit systems and discrete/hybrid controller implementations. It is also used as the machine-control backend for `ElectricMachineWinch.jl`, which integrates the induction-machine model inside a `KiteSimulators.jl` autopilot simulation.

## Purpose

`IM_AWES_bench.jl` provides reusable induction-machine models, observers, controllers, estimators, and standalone benchmark simulators. The current focus is a squirrel-cage induction machine controlled with field-oriented control (FOC), including speed-mode and torque-mode outer loops.

The package is intended to support two workflows:

1. **Standalone machine-control testing**: simulate scalar control, current-loop FOC, torque FOC, and speed FOC directly inside this package.
2. **KiteSimulators integration**: provide validated discrete FOC blocks and machine parameters to `ElectricMachineWinch.jl`, which acts as a bridge between the electrical machine and the kite/winch mechanical simulation.

## Main model convention

The induction-machine plant uses stationary alpha-beta electrical variables and the following mechanical sign convention:

```text
J*dωm/dt = Te + TL - B*ωm
```

where:

- `ωm` is machine mechanical speed `[rad/s]`.
- `Te` is electromagnetic torque `[Nm]`.
- `TL` or `Tload` is external load torque `[Nm]`.
- `B*ωm` is viscous friction torque.
- Positive `TL` pulls the shaft toward positive speed.

This convention is used consistently in the plant, the speed controller load feedforward, and the load-torque Kalman estimator.

## Package organization

```text
IM_AWES_bench.jl/
├── Project.toml
├── README.md
└── src/
    ├── IM_AWES_bench.jl
    ├── profiles/
    │   ├── frequency_profiles.jl
    │   └── load_profiles.jl
    ├── controls/
    │   ├── scalar_vf_control.jl
    │   └── FOC/
    │       ├── current_controller.jl
    │       ├── current_controller_discrete.jl
    │       ├── outer_torque_flux_f1_discrete.jl
    │       └── outer_speed_flux_f1_discrete.jl
    ├── estimators/
    │   ├── rotor_flux_observer.jl
    │   ├── rotor_flux_observer_discrete.jl
    │   └── load_torque_kalman_discrete.jl
    ├── plants/
    │   └── induction_machine_alpha_beta.jl
    ├── systems/
    │   ├── scalar_im_system.jl
    │   └── foc_current_im_system.jl
    └── simulators/
        ├── hybrid_foc_current_simulator.jl
        ├── hybrid_foc_torque_f1_simulator.jl
        └── hybrid_foc_speed_f1_simulator.jl
```

## Main components

### Induction-machine alpha-beta plant

The plant is implemented in:

```text
src/plants/induction_machine_alpha_beta.jl
```

The plant inputs are:

- `vsα`, `vsβ`: stator voltage components in the stationary alpha-beta frame.
- `Tload_cmd`: external mechanical load torque.

The plant states are:

- `isα`, `isβ`: stator currents.
- `irα`, `irβ`: rotor currents.
- `ωm`: mechanical speed.
- `θm`: mechanical angle.

The main plant outputs are:

- `Te`: electromagnetic torque.
- `n_rpm`: mechanical speed in rpm.
- flux variables and auxiliary speed outputs.

The plant computes the electromagnetic torque as:

```text
Te = 1.5*p*(ψsα*isβ - ψsβ*isα)
```

and includes the mechanical equation:

```text
J*dωm/dt = Te + Tload_cmd - B*ωm
```

### Discrete rotor-flux observer

Implemented in:

```text
src/estimators/rotor_flux_observer_discrete.jl
```

The observer takes measured alpha-beta currents, mechanical angle, and mechanical speed, then computes:

- estimated rotor flux in the rotor and flux frames;
- estimated electrical angle `theta_e`;
- observed `isd`, `isq` in the rotor-flux frame;
- estimated electromagnetic torque;
- slip and synchronous electrical speed `omega_e`.

This observer is used by the discrete FOC loops.

### Discrete current controller

Implemented in:

```text
src/controls/FOC/current_controller_discrete.jl
```

Inputs:

- `isd_ref`, `isq_ref`;
- measured/observed `isd_med`, `isq_med`;
- `omega_e`;
- `lambda_rd`;
- controller limits `Vs_max`, `Is_max`.

Outputs:

- `vsd`, `vsq`;
- limited current references;
- saturation flag;
- diagnostic voltages and errors.

The current controller applies:

- optional current measurement filtering;
- d-axis-priority current limiting;
- PI current regulation;
- optional decoupling/feedforward terms;
- voltage-vector saturation using `Vs_max`;
- antiwindup when voltage saturation occurs.

### Outer torque/flux F1 controller

Implemented in:

```text
src/controls/FOC/outer_torque_flux_f1_discrete.jl
```

This controller receives an external torque reference `Te_ref_ext`, applies torque ramping and saturation, and generates:

```text
isd_ref, isq_ref
```

using a nominal F1 flux reference and a T1 torque-to-current conversion.

This is useful for torque-control tests where the outer speed loop is not required.

### Outer speed/flux F1 controller

Implemented in:

```text
src/controls/FOC/outer_speed_flux_f1_discrete.jl
```

This is the main controller used by `ElectricMachineWinch.jl` during KiteSimulators integration.

It receives:

- `wm_ref`: machine speed reference `[rad/s]`;
- `wm_med`: measured machine speed `[rad/s]`;
- `TL_est`: estimated/measured load torque `[Nm]`.

It computes:

- filtered speed `wm_filt`;
- ramp-limited speed reference `wm_ref_ramp`;
- speed PI torque request;
- optional load feedforward;
- saturated torque reference `Te_ref_out`;
- F1 d-axis flux-producing current reference `isd_ref`;
- q-axis torque-producing current reference `isq_ref`.

#### Field weakening

The speed/flux controller includes optional speed-based field weakening:

```julia
use_field_weakening::Bool = false
wm_base_fw::Float64 = 120.0
```

When enabled, the d-axis reference is reduced above `wm_base_fw`:

```text
isd_ref ≈ isd_nom * wm_base_fw / max(abs(wm_filt), abs(wm_ref_ramp))
```

with lower bound `isd_min`.

The controller then recomputes the torque constant from the weakened flux:

```text
lambda_ref_out = Lm * isd_ref
Kt_ref = 1.5 * p * (Lm/Lrr) * lambda_ref_out
isq_ref_unsat = Te_ref_out / Kt_ref
```

This is important because the nominal torque constant is no longer valid once the field is weakened.

### Load torque Kalman estimator

Implemented in:

```text
src/estimators/load_torque_kalman_discrete.jl
```

The estimator uses the same mechanical convention:

```text
J*dω/dt = Te + TL - B*ω
```

with state:

```text
x = [ω_hat, TL_hat]
```

and measurement:

```text
y = ω
```

It can be used in the standalone speed FOC simulator to estimate the load torque instead of using the actual load directly.

## Available standalone simulators

### Current-loop FOC simulator

```julia
simulate_foc_current_hybrid(; kwargs...)
```

Tests the inner FOC current controller and rotor-flux observer with current references and load profiles.

### Torque FOC simulator

```julia
simulate_foc_torque_f1_hybrid(; kwargs...)
```

Tests torque-mode FOC:

```text
Te_ref profile
    -> outer torque/flux F1
    -> current controller
    -> alpha-beta IM plant
```

### Speed FOC simulator

```julia
simulate_foc_speed_f1_hybrid(; kwargs...)
```

Tests speed-mode FOC:

```text
wm_ref profile
    -> outer speed/flux F1
    -> current controller
    -> alpha-beta IM plant
```

This simulator supports:

- internal speed ramps;
- external speed/load profiles;
- internal or profile-based load torque;
- optional load estimator choices: `:none`, `:actual`, `:kalman`;
- parameter mismatch scaling for plant, observer, and controller;
- current and voltage saturation diagnostics.

## Exports

The top-level module exports the main simulation functions and reusable controller/observer types:

```julia
using IM_AWES_bench

simulate_scalar_im
simulate_foc_current_im
simulate_foc_current_hybrid
simulate_foc_torque_f1_hybrid
simulate_foc_speed_f1_hybrid

CurrentControllerDiscreteState
CurrentControllerDiscreteParams
current_controller_step!

RotorFluxObserverDiscreteState
RotorFluxObserverDiscreteParams
rotor_flux_observer_step!

LoadTorqueKalmanState
LoadTorqueKalmanParams
load_torque_kalman_step!

OuterTorqueFluxF1State
OuterTorqueFluxF1Params
outer_torque_flux_f1_step!

OuterSpeedFluxF1State
OuterSpeedFluxF1Params
outer_speed_flux_f1_step!
```

## Example: speed FOC test

```julia
using IM_AWES_bench

res = simulate_foc_speed_f1_hybrid(
    t_end = 12.0,
    Ts = 100e-6,
    plant_substeps = 1,
    wm_ref_high_rpm = 500.0,
    load_profile = :steps,
    use_load_feedforward = true,
)
```

The returned object is a named tuple containing time histories such as:

```text
t
wm_ref
wm_ref_ramp
wm_filt
Te_ref_out
isd_ref
isq_ref
isd
isq
vsd
vsq
torque
Tload
Pelec
Pmech
saturation_current
sat_isd
sat_isq
sat_Te
```

## Relationship with ElectricMachineWinch.jl

`IM_AWES_bench.jl` should remain the package that owns the electrical-machine control logic.

`ElectricMachineWinch.jl` should remain a bridge package. It imports this package and calls:

```julia
rotor_flux_observer_step!
outer_speed_flux_f1_step!
current_controller_step!
```

inside its `FOCSpeedF1Controller` wrapper.

For that reason, long-term machine-control features such as field weakening, torque/current allocation, flux scheduling, load-torque estimation, or new FOC modes should be implemented here first, then exposed through `ElectricMachineWinch.jl` only as constructor options.

## Notes for reproducibility

After changing exported structs such as:

```julia
OuterSpeedFluxF1Params
OuterSpeedFluxF1Output
CurrentControllerDiscreteParams
```

a full Julia restart is recommended. `Revise.jl` may update function bodies, but it cannot reliably update struct layouts in an active session.

To verify that the correct local package is being used:

```julia
using IM_AWES_bench
pathof(IM_AWES_bench)
fieldnames(IM_AWES_bench.OuterSpeedFluxF1Params)
```

For the KiteSimulators integration, `fieldnames(IM_AWES_bench.OuterSpeedFluxF1Params)` should include:

```julia
:use_field_weakening
:wm_base_fw
```
