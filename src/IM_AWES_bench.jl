module IM_AWES_bench
"""
bla
"""

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
# Calling one of these without `using ModelingToolkit` raises a MethodError
# listing zero methods, which is the intended signal.

function build_scalar_im_system end
function simulate_scalar_im end
function build_foc_current_im_system end
function simulate_foc_current_im end

# Backward-compatible alias, so old scripts do not immediately break. It has to
# be declared here rather than in the extension, for the same binding reason.
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