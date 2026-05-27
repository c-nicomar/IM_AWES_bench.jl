module IM_AWES_bench_jl

using ModelingToolkit
using ModelingToolkit: t_nounits as t, D_nounits as D
using OrdinaryDiffEq

include("profiles/frequency_profiles.jl")
include("profiles/load_profiles.jl")
include("controls/scalar_vf_control.jl")
include("plants/induction_machine_alpha_beta.jl")
include("estimators/rotor_flux_observer.jl")
include("systems/scalar_im_system.jl")

export build_scalar_im_system
export build_scalar_im_model
export simulate_scalar_im

end