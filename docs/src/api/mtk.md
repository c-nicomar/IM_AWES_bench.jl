# ModelingToolkit extension

Everything on this page lives in `InductionMachineDrivesMTKExt` and exists only when
**both** `ModelingToolkit` and `OrdinaryDiffEq` are loaded:

```julia
using ModelingToolkit, OrdinaryDiffEq, InductionMachineDrives
```

Julia loads an extension only once *every* package in its trigger list is
present. `using ModelingToolkit` alone leaves the four public builders at zero
methods, and calling one then raises a `MethodError` listing no methods — that
means a trigger is missing, not that the function is broken.

```@autodocs
Modules = [Base.get_extension(InductionMachineDrives, :InductionMachineDrivesMTKExt)]
Pages   = ["InductionMachineDrivesMTKExt.jl"]
```

## Public system builders

These four names are declared as empty stubs in `src/InductionMachineDrives.jl` and get
their methods here. A package extension can add *methods* to a function its
parent already declares, but it cannot introduce a new binding into the parent's
namespace — which is why the stubs exist at all.

```@docs
InductionMachineDrives.build_scalar_im_system
InductionMachineDrives.simulate_scalar_im
InductionMachineDrives.build_foc_current_im_system
InductionMachineDrives.simulate_foc_current_im
```

`build_scalar_im_model` is a backward-compatible `const` alias for
`build_scalar_im_system`, declared in the parent module for the same binding
reason.

## Reference profiles

```@autodocs
Modules = [Base.get_extension(InductionMachineDrives, :InductionMachineDrivesMTKExt)]
Pages   = ["profiles/frequency_profiles.jl", "profiles/load_profiles.jl"]
```

## Controls

```@autodocs
Modules = [Base.get_extension(InductionMachineDrives, :InductionMachineDrivesMTKExt)]
Pages   = ["controls/scalar_vf_control.jl", "controls/FOC/current_controller.jl"]
```

## Plant

```@autodocs
Modules = [Base.get_extension(InductionMachineDrives, :InductionMachineDrivesMTKExt)]
Pages   = ["plants/induction_machine_alpha_beta.jl"]
```

## Observer

```@autodocs
Modules = [Base.get_extension(InductionMachineDrives, :InductionMachineDrivesMTKExt)]
Pages   = ["estimators/rotor_flux_observer.jl"]
```

!!! note "The `D` alias"

    The equation builders above write `D(x) ~ ...` without ever naming
    ModelingToolkit. They rely on the `D_nounits as D` import at the top of
    `ext/InductionMachineDrivesMTKExt.jl`. Moving a builder out of that module without
    carrying the import fails at parse time.
