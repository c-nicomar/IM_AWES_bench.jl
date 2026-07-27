"""
    IM_AWES_benchMTKExt

ModelingToolkit extension for IM_AWES_bench.

Everything symbolic lives here: the continuous equation builders and the two
acausal system assemblies. It loads automatically once both ModelingToolkit and
OrdinaryDiffEq are present in the session, so

    using IM_AWES_bench                                        # hybrid simulators only, no MTK
    using IM_AWES_bench, ModelingToolkit, OrdinaryDiffEq       # + the MTK system builders

Both triggers are required. Julia loads an extension only once *every* package
in its trigger list is present, so `using ModelingToolkit` alone leaves the
builders at zero methods — OrdinaryDiffEq is not optional here, because the
`simulate_*` functions default to `Rodas5P()` and call `solve`.

The hybrid FOC simulators in `src/simulators/` are plain Julia over concrete
`Float64` state and never touch symbolic types, which is what makes this split
possible — see Plan.md.

Note on `D`: the equation builders write `D(x) ~ ...` without ever naming
ModelingToolkit. They depend on the `D_nounits as D` alias imported below, which
used to live at the top of the main module. Moving a builder out of this module
without carrying that import along will fail at parse time.
"""
module IM_AWES_benchMTKExt

using IM_AWES_bench

using ModelingToolkit
using ModelingToolkit: t_nounits as t, D_nounits as D
using OrdinaryDiffEq

# Methods are added to the stubs declared in the main module. Only these four
# are part of the public API; the equation builders below are internal to this
# extension and are deliberately not declared in the parent.
import IM_AWES_bench:
    build_scalar_im_system,
    simulate_scalar_im,
    build_foc_current_im_system,
    simulate_foc_current_im

# ============================================================
# Profiles
# ============================================================

include("profiles/frequency_profiles.jl")
include("profiles/load_profiles.jl")

# ============================================================
# Continuous control blocks
# ============================================================

include("controls/scalar_vf_control.jl")
include("controls/FOC/current_controller.jl")

# ============================================================
# Plants
# ============================================================

include("plants/induction_machine_alpha_beta.jl")

# ============================================================
# Estimators
# ============================================================

include("estimators/rotor_flux_observer.jl")

# ============================================================
# Systems
# ============================================================

include("systems/scalar_im_system.jl")
include("systems/foc_current_im_system.jl")

end # module
