# IM AWES Bench Julia Model

[![Dev](https://img.shields.io/badge/docs-dev-blue.svg)](https://c-nicomar.github.io/IM_AWES_bench.jl/dev/)
[![Tests](https://github.com/c-nicomar/IM_AWES_bench.jl/actions/workflows/Tests.yml/badge.svg)](https://github.com/c-nicomar/IM_AWES_bench.jl/actions/workflows/Tests.yml)

Julia modelling project for reproducing the induction-machine AWES bench simulation logic from the Simulink model.

## Goal

Develop a Julia-based model of the induction machine, electrical supply/control interface, and mechanical bench dynamics for AWES ground-station studies.

## Detailed documentation

- [Project documentation](docs/src/architecture.md) — architecture, sign conventions, and the full description of controllers, estimators, and simulators.
- [Package overview](docs/src/overview.md) — purpose, supported workflows, and use as the machine-control backend for `ElectricMachineWinch.jl`.
- [MTPA integration](docs/src/mtpa.md) — the constrained-MTPA speed controller, its files, and the include-order constraint.
- [160 kW FOC F1 simulation](docs/src/foc_f1_160kw.md) — bipolar speed tracking, field weakening, and voltage/current limits on a 160 kW machine.


