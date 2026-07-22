module IM_AWES_bench
"""
bla
"""

using ModelingToolkit
using ModelingToolkit: t_nounits as t, D_nounits as D
using OrdinaryDiffEq

# ============================================================
# Profiles
# ============================================================

include("profiles/frequency_profiles.jl")
include("profiles/load_profiles.jl")

# ============================================================
# Continuous / MTK control blocks
# ============================================================

include("controls/scalar_vf_control.jl")
include("controls/FOC/current_controller.jl")

# ============================================================
# Discrete FOC control blocks
# ============================================================

include("controls/FOC/current_controller_discrete.jl")
include("controls/FOC/outer_torque_flux_f1_discrete.jl")
include("controls/FOC/outer_speed_flux_f1_discrete.jl")

# ============================================================
# Plants
# ============================================================

include("plants/induction_machine_alpha_beta.jl")

# ============================================================
# Estimators
# ============================================================

include("estimators/rotor_flux_observer.jl")
include("estimators/rotor_flux_observer_discrete.jl")
include("estimators/load_torque_kalman_discrete.jl")

# ============================================================
# MTK systems
# ============================================================

include("systems/scalar_im_system.jl")
include("systems/foc_current_im_system.jl")

# ============================================================
# Hybrid simulators
# ============================================================

include("simulators/hybrid_foc_current_simulator.jl")
include("simulators/hybrid_foc_torque_f1_simulator.jl")
include("simulators/hybrid_foc_speed_f1_simulator.jl")

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

end