module IM_AWES_bench_jl

using ModelingToolkit
using ModelingToolkit: t_nounits as t, D_nounits as D
using OrdinaryDiffEq

include("profiles/frequency_profiles.jl")
include("profiles/load_profiles.jl")

include("controls/scalar_vf_control.jl")
include("controls/FOC/current_controller.jl")

include("plants/induction_machine_alpha_beta.jl")
include("estimators/rotor_flux_observer.jl")

include("systems/scalar_im_system.jl")
include("systems/foc_current_im_system.jl")

include("controls/FOC/current_controller_discrete.jl")
include("estimators/rotor_flux_observer_discrete.jl")
include("simulators/hybrid_foc_current_simulator.jl")

include("controls/FOC/outer_torque_flux_f1_discrete.jl")
include("simulators/hybrid_foc_torque_f1_simulator.jl")

export build_scalar_im_system
export build_scalar_im_model
export simulate_scalar_im

export build_foc_current_im_system
export simulate_foc_current_im

end