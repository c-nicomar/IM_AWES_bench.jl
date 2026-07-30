# Simulators

```@meta
CurrentModule = InductionMachineDrives
```

The hybrid simulators step the hand-written plant derivative alongside the
discrete blocks at a fixed sample time. They are plain Julia over concrete
`Float64` and never touch symbolic types — that is what keeps ModelingToolkit
out of `src/`.

## Current-loop simulator

```@autodocs
Modules = [InductionMachineDrives]
Pages   = ["simulators/hybrid_foc_current_simulator.jl"]
```

## Torque-loop simulator, F1 constant flux

```@autodocs
Modules = [InductionMachineDrives]
Pages   = ["simulators/hybrid_foc_torque_f1_simulator.jl"]
```

## Speed-loop simulator, F1 constant flux

This file also defines `interp_profile_linear` and the other CSV
profile-playback helpers, which the MTPA simulator reuses — hence the include
order fixed in `src/InductionMachineDrives.jl`.

```@autodocs
Modules = [InductionMachineDrives]
Pages   = ["simulators/hybrid_foc_speed_f1_simulator.jl"]
```

## Speed-loop simulator, constrained MTPA

```@autodocs
Modules = [InductionMachineDrives]
Pages   = ["simulators/hybrid_foc_speed_mtpa_simulator.jl"]
```

## 160 kW speed-loop simulator

The bipolar speed-tracking case with field weakening and voltage/current limits;
see [160 kW FOC F1 case](../foc_f1_160kw.md) for the full description.

```@autodocs
Modules = [InductionMachineDrives]
Pages   = ["simulators/hybrid_foc_speed_f1_160kw_simulator.jl"]
```
