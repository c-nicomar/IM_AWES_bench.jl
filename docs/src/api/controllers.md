# Controllers

```@meta
CurrentModule = InductionMachineDrives
```

The discrete FOC control blocks. Each follows the same convention: a
`*State` / `*Params` / `*Output` struct trio plus a mutating `*_step!` function,
called once per fixed-step iteration of the hybrid loop.

## Inner current controller

Runs closest to the machine, turning dq current references into a dq voltage
command.

```@autodocs
Modules = [InductionMachineDrives]
Pages   = ["controls/FOC/current_controller_discrete.jl"]
```

## Outer torque loop, F1 constant flux

```@autodocs
Modules = [InductionMachineDrives]
Pages   = ["controls/FOC/outer_torque_flux_f1_discrete.jl"]
```

## Outer speed loop, F1 constant flux

```@autodocs
Modules = [InductionMachineDrives]
Pages   = ["controls/FOC/outer_speed_flux_f1_discrete.jl"]
```

## Outer speed loop, constrained MTPA with field weakening

See [MTPA integration](../mtpa.md) for the include-order constraint that this
controller imposes on `src/InductionMachineDrives.jl`.

```@autodocs
Modules = [InductionMachineDrives]
Pages   = ["controls/FOC/outer_speed_flux_mtpa_discrete.jl"]
```
