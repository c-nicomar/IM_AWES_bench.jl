# Estimators

```@meta
CurrentModule = InductionMachineDrives
```

Both estimators are discrete and are stepped once per fixed-step iteration, the
observer before the outer loop and the Kalman estimator alongside it.

!!! warning "Sign convention"

    The load-torque Kalman estimator is built around
    `J * dω/dt = Te + TL - B * ω`, which fixes `A12 = Ts/J` — *not* `-Ts/J`. See
    sections 9 and 11 of [the architecture page](../architecture.md).

## Rotor-flux and torque observer

```@autodocs
Modules = [InductionMachineDrives]
Pages   = ["estimators/rotor_flux_observer_discrete.jl"]
```

## Load-torque Kalman estimator

```@autodocs
Modules = [InductionMachineDrives]
Pages   = ["estimators/load_torque_kalman_discrete.jl"]
```
