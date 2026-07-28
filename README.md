# IM AWES Bench Julia Model

Julia modelling project for reproducing the induction-machine AWES bench simulation logic from the Simulink model.

## Goal

Develop a Julia-based model of the induction machine, electrical supply/control interface, and mechanical bench dynamics for AWES ground-station studies.

## Detailed documentation

- [Project documentation](docs/README_IM_AWES_bench_jl.md) — architecture, sign conventions, and the full description of controllers, estimators, and simulators.
- [Package overview](docs/README_IM_AWES_bench_updated.md) — purpose, supported workflows, and use as the machine-control backend for `ElectricMachineWinch.jl`.
- [MTPA integration](docs/README_MTPA_INTEGRATION.md) — the constrained-MTPA speed controller, its files, and the include-order constraint.
- [160 kW FOC F1 simulation](docs/README_FOC_F1_160KW.md) — bipolar speed tracking, field weakening, and voltage/current limits on a 160 kW machine.


