"""
    IM_AWES_bench

Reproduction of an induction-machine (IM) AWES ground-station bench simulation,
originally a Simulink model. It offers two modelling paths.

**Hybrid FOC path** — the package core, and the only part with no dependencies.
A hand-written induction-machine plant is integrated with fixed-step RK4 while
hand-written discrete controllers and observers run alongside it at a fixed
sample time `Ts`, which is close to how a real embedded implementation behaves.
Everything is concrete `Float64`; nothing is symbolic. Entry points:

- [`simulate_foc_current_hybrid`](@ref) — inner current loop only.
- [`simulate_foc_torque_f1_hybrid`](@ref) — F1 constant-flux torque control.
- [`simulate_foc_speed_f1_hybrid`](@ref) — F1 constant-flux speed control,
  optional load-torque estimation and CSV profile playback.
- [`simulate_foc_speed_mtpa_hybrid`](@ref) — constrained-MTPA speed control.
- [`simulate_foc_speed_f1_im_160kw`](@ref) — the 160 kW machine case.

The discrete blocks these simulators are built from are exported too, so they
can be driven directly from an external loop — for instance as the machine
control backend of `ElectricMachineWinch.jl`. Each follows the same convention:
a `*State` / `*Params` / `*Output` struct trio plus a `*_step!` function that
mutates the state and returns a fresh output
([`current_controller_step!`](@ref), [`rotor_flux_observer_step!`](@ref),
[`load_torque_kalman_step!`](@ref), [`outer_torque_flux_f1_step!`](@ref),
[`outer_speed_flux_f1_step!`](@ref), [`outer_speed_flux_mtpa_step!`](@ref)).

**Scalar V/f and symbolic FOC path** — ModelingToolkit systems used for basic
plant validation. These live in the `IM_AWES_benchMTKExt` package extension and
only acquire methods once *both* triggers are loaded:

```julia
using ModelingToolkit, OrdinaryDiffEq, IM_AWES_bench
```

Calling [`build_scalar_im_system`](@ref), [`simulate_scalar_im`](@ref),
[`build_foc_current_im_system`](@ref) or [`simulate_foc_current_im`](@ref)
without both raises a `MethodError` listing zero methods; that is the intended
signal, not a bug.

# Sign convention

The mechanical equation used throughout the plant, the estimators and every
feedforward term is

```
J * dω/dt = Te + TL - B * ω
```

so electromagnetic torque `Te` and load torque `TL` are both positive when they
accelerate the machine toward positive speed, while friction `B*ω` opposes
motion. A positive `TL` therefore *helps* positive acceleration, which is why
load feedforward enters the speed loop with a negative sign.
"""
module IM_AWES_bench

# ============================================================
# ModelingToolkit surface
# ============================================================
#
# The symbolic model builders live in ext/IM_AWES_benchMTKExt.jl and load only
# when ModelingToolkit and OrdinaryDiffEq are both present in the session. These
# stubs exist so the names can be declared and exported from here: a package
# extension can add *methods* to a function its parent already declares, but it
# cannot introduce a new binding into the parent's namespace.
#
# Calling one of these before both triggers are loaded raises a MethodError
# listing zero methods, which is the intended signal. Loading ModelingToolkit
# alone is not enough — Julia waits for every package in the trigger list.

function build_scalar_im_system end
function simulate_scalar_im end
function build_foc_current_im_system end
function simulate_foc_current_im end

# Backward-compatible alias, so old scripts do not immediately break. It has to
# be declared here rather than in the extension, for the same binding reason.
"""
    build_scalar_im_model(; kwargs...)

Deprecated alias for [`build_scalar_im_system`](@ref), kept so scripts written
before the rename keep working. It is the very same function object, so it
carries the same methods and the same extension requirement: both
`ModelingToolkit` and `OrdinaryDiffEq` must be loaded before it has any methods.
Prefer `build_scalar_im_system` in new code.
"""
const build_scalar_im_model = build_scalar_im_system

# ============================================================
# Discrete FOC control blocks
# ============================================================

include("controls/FOC/current_controller_discrete.jl")
include("controls/FOC/outer_torque_flux_f1_discrete.jl")
include("controls/FOC/outer_speed_flux_f1_discrete.jl")
include("controls/FOC/outer_speed_flux_mtpa_discrete.jl")

# ============================================================
# Estimators
# ============================================================

include("estimators/rotor_flux_observer_discrete.jl")
include("estimators/load_torque_kalman_discrete.jl")

# ============================================================
# Hybrid simulators
# ============================================================

include("simulators/hybrid_foc_current_simulator.jl")
include("simulators/hybrid_foc_torque_f1_simulator.jl")
include("simulators/hybrid_foc_speed_f1_simulator.jl")
include("simulators/hybrid_foc_speed_mtpa_simulator.jl")
include("simulators/hybrid_foc_speed_f1_160kw_simulator.jl")

# ============================================================
# Exports: scalar / MTK systems
# ============================================================

export build_scalar_im_system
export build_scalar_im_model
export simulate_scalar_im

export build_foc_current_im_system
export simulate_foc_current_im

# ============================================================
# Exports: hybrid FOC simulators
# ============================================================

export simulate_foc_current_hybrid
export simulate_foc_torque_f1_hybrid
export simulate_foc_speed_f1_hybrid
export simulate_foc_speed_mtpa_hybrid

# ============================================================
# Exports: useful discrete estimator/controller types
# ============================================================

export CurrentControllerDiscreteState
export CurrentControllerDiscreteParams
export CurrentControllerDiscreteOutput
export current_controller_step!

export RotorFluxObserverDiscreteState
export RotorFluxObserverDiscreteParams
export RotorFluxObserverDiscreteOutput
export rotor_flux_observer_step!

export LoadTorqueKalmanState
export LoadTorqueKalmanParams
export LoadTorqueKalmanOutput
export load_torque_kalman_step!

export OuterTorqueFluxF1State
export OuterTorqueFluxF1Params
export OuterTorqueFluxF1Output
export outer_torque_flux_f1_step!

export OuterSpeedFluxF1State
export OuterSpeedFluxF1Params
export OuterSpeedFluxF1Output
export outer_speed_flux_f1_step!

export OuterSpeedFluxMTPAState
export OuterSpeedFluxMTPAParams
export OuterSpeedFluxMTPAOutput
export outer_speed_flux_mtpa_step!

export simulate_foc_speed_f1_im_160kw
export im_160kw_load_torque_profile
export im_160kw_speed_reference

end