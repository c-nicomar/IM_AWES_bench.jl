# Constrained-MTPA speed-controller integration

Add the files to the repository using this layout:

```text
InductionMachineDrives.jl/
├── scripts/
│   └── run_foc_speed_mtpa_awes_profile.jl
├── src/
│   ├── controls/
│   │   └── FOC/
│   │       └── outer_speed_flux_mtpa_discrete.jl
│   └── simulators/
│       └── hybrid_foc_speed_mtpa_simulator.jl
└── test/
    ├── test_outer_speed_flux_mtpa_discrete.jl
    └── test_foc_speed_mtpa_simulator.jl
```

Then apply the edits shown in `InductionMachineDrives_mtpa_integration.patch`.

## Run the AWES example

From the repository root:

```julia
include("scripts/run_foc_speed_mtpa_awes_profile.jl")
```

Or from a terminal:

```text
julia --project=scripts scripts/run_foc_speed_mtpa_awes_profile.jl
```

The runner uses the existing profile:

```text
profiles/delta_kite_13_ms_profiles.csv
```

and writes:

```text
results/foc_speed_mtpa_awes_profile.csv
```

## Main MTPA settings

```julia
lambda_rd_floor = 0.35
Te_reserve = 45.0
id_dot_max = 600.0
Is_max = 40.0
```

For a fair F1-versus-MTPA comparison, keep the same machine parameters,
current-controller settings, estimator settings, current and voltage limits,
sampling period, and AWES input profile.

## Important include order

`hybrid_foc_speed_mtpa_simulator.jl` reuses the profile interpolation helpers
defined in `hybrid_foc_speed_f1_simulator.jl`. Therefore, the F1 simulator must
remain included first in `src/InductionMachineDrives.jl`.

## Tests

From the repository root:

```julia
using Pkg
Pkg.activate(".")
Pkg.test()
```
