# IM_AWES_bench_jl

This repository contains a first Julia implementation of an induction-machine bench model intended for later use in airborne wind energy system (AWES) electromechanical simulations.

The current model reproduces a simplified version of a Simulink/Simscape benchmark where a squirrel-cage induction machine is driven by an ideal three-phase voltage source using open-loop scalar V/f control.

The implementation is intentionally simple at this stage: it focuses on reproducing the main electrical and mechanical dynamics before adding more detailed converter, controller, winch, or AWES load models.

---

## Repository structure

```text
IM_AWES_bench_jl/
├── Project.toml
├── Manifest.toml
├── README.md
├── src/
│   └── IM_AWES_bench_jl.jl
├── examples/
│   └── run_scalar_im.jl
├── data/
├── results/
└── scripts/
```

The intended use of each folder is:

| Folder | Purpose |
|---|---|
| `src/` | Reusable model definitions and simulation functions |
| `examples/` | Small runnable examples |
| `data/` | Input data, profiles, MATLAB exports, measured traces |
| `results/` | Simulation outputs, CSV files, plots |
| `scripts/` | Longer analysis or post-processing scripts |

---

## Current model

The file

```text
src/IM_AWES_bench_jl.jl
```

defines a module called:

```julia
IM_AWES_bench_jl
```

At the moment, it exports two main functions:

```julia
build_scalar_im_model()
simulate_scalar_im()
```

The model represents a squirrel-cage induction machine in the stationary `αβ` reference frame.

The simulated system is:

```text
scalar V/f controller
        ↓
ideal balanced three-phase voltage source
        ↓
abc to αβ transformation
        ↓
squirrel-cage induction machine model
        ↓
one-mass mechanical equation with inertia, friction, and load torque
```

The model does not yet include:

- PWM switching,
- a DC-link inverter,
- electrical pins in full acausal Simscape style,
- closed-loop field-oriented control,
- a winch/drum model,
- tether force dynamics,
- kite or AWES aerodynamic model.

Those will be added later once the basic induction-machine model is validated.

---

## Induction machine equations

The model uses stationary-frame `αβ` induction machine equations.

The stator voltage equations are:

```text
v_sα = R_s i_sα + dψ_sα/dt
v_sβ = R_s i_sβ + dψ_sβ/dt
```

The squirrel-cage rotor voltage equations are:

```text
0 = R_r i_rα + dψ_rα/dt + p ω_m ψ_rβ
0 = R_r i_rβ + dψ_rβ/dt - p ω_m ψ_rα
```

The flux-current relations are:

```text
ψ_sα = L_s i_sα + L_m i_rα
ψ_sβ = L_s i_sβ + L_m i_rβ

ψ_rα = L_m i_sα + L_r i_rα
ψ_rβ = L_m i_sβ + L_r i_rβ
```

The electromagnetic torque is computed as:

```text
T_e = 3/2 p (ψ_sα i_sβ - ψ_sβ i_sα)
```

The mechanical equation is:

```text
J dω_m/dt = T_e - T_load - Bω_m
```

where:

| Symbol | Meaning |
|---|---|
| `R_s` | stator resistance |
| `R_r` | rotor resistance |
| `L_s` | stator inductance |
| `L_r` | rotor inductance |
| `L_m` | magnetizing inductance |
| `p` | pole pairs |
| `ω_m` | mechanical rotor speed |
| `T_e` | electromagnetic torque |
| `T_load` | external load torque |
| `J` | total inertia |
| `B` | viscous friction coefficient |

---

## Scalar V/f control

The current example uses open-loop scalar V/f control.

For a reference frequency `f_ref`, the electrical angle is obtained from:

```text
dθ_s/dt = 2π f_ref
```

The phase voltage peak is scaled proportionally to frequency:

```text
V_phase,peak = √2 · V_LL,nom / √3 · f_ref / f_nom
```

The three phase voltage references are:

```text
v_a = V_phase,peak sin(θ_s)
v_b = V_phase,peak sin(θ_s - 2π/3)
v_c = V_phase,peak sin(θ_s + 2π/3)
```

For the current example:

```text
f_ref = 25 Hz
V_LL,nom = 380 V
f_nom = 50 Hz
p = 2
```

The expected synchronous mechanical speed is therefore:

```text
n_sync = 60 f_ref / p = 60 · 25 / 2 = 750 rpm
```

With zero load torque, the rotor speed should approach approximately this value after the transient.

---

## Running the example

The main example is:

```text
examples/run_scalar_im.jl
```

From the project root, start Julia with the local project activated:

```bash
julia --project=.
```

Then run:

```julia
include("examples\\run_scalar_im.jl")
```

Alternatively, from the terminal:

```bash
julia --project=. examples/run_scalar_im.jl
```

The example simulates:

```julia
sol, sys = IM_AWES_bench_jl.simulate_scalar_im(
    tspan = (0.0, 10.0),
    f_ref_val = 25.0,
    Tload_val = 0.0,
)
```

This corresponds to a 10-second simulation with:

| Parameter | Value |
|---|---|
| Frequency reference | `25 Hz` |
| Load torque | `0 N m` |
| Pole pairs | `2` |
| Expected synchronous speed | `750 rpm` |

---

## What the example plots

The example uses `ControlPlots.jl` to visualize:

1. rotor speed in rpm,
2. synchronous speed reference,
3. electromagnetic torque.

The plotted signals are:

```julia
speed = sol[sys.n_rpm]
torque = sol[sys.Te]
speed_sync = fill(n_sync, length(sol.t))
```

The speed plot is useful to verify that the machine accelerates toward the expected synchronous speed.

The torque plot is useful to inspect the startup transient and electromagnetic torque oscillations.

---

## Saved results

The example also saves the main simulation variables to a CSV file:

```text
results/scalar_im_25Hz_results.csv
```

The CSV file contains:

| Column | Meaning |
|---|---|
| `t_s` | simulation time |
| `speed_rpm` | rotor speed in rpm |
| `speed_sync_rpm` | synchronous speed reference |
| `torque_Nm` | electromagnetic torque |
| `omega_m_rad_s` | mechanical rotor speed in rad/s |
| `va_V`, `vb_V`, `vc_V` | phase voltage references |
| `vs_alpha_V`, `vs_beta_V` | stationary-frame stator voltage |
| `is_alpha_A`, `is_beta_A` | stationary-frame stator currents |
| `ir_alpha_A`, `ir_beta_A` | stationary-frame rotor currents |
| `psi_s_alpha_Wb`, `psi_s_beta_Wb` | stator flux components |
| `psi_r_alpha_Wb`, `psi_r_beta_Wb` | rotor flux components |

CSV is used instead of `.mat` because it can be opened easily in Julia, MATLAB, Python, Excel, and most data-analysis tools.

---

## Opening the results in MATLAB

From MATLAB:

```matlab
data = readtable("results/scalar_im_25Hz_results.csv");

figure
plot(data.t_s, data.speed_rpm)
hold on
plot(data.t_s, data.speed_sync_rpm, "--")
grid on
xlabel("Time [s]")
ylabel("Speed [rpm]")
legend("Rotor speed", "Synchronous speed")

figure
plot(data.t_s, data.torque_Nm)
grid on
xlabel("Time [s]")
ylabel("Torque [N m]")
```

---

## Opening the results in Python

From Python:

```python
import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("results/scalar_im_25Hz_results.csv")

plt.figure()
plt.plot(data["t_s"], data["speed_rpm"], label="Rotor speed")
plt.plot(data["t_s"], data["speed_sync_rpm"], "--", label="Synchronous speed")
plt.grid(True)
plt.xlabel("Time [s]")
plt.ylabel("Speed [rpm]")
plt.legend()

plt.figure()
plt.plot(data["t_s"], data["torque_Nm"])
plt.grid(True)
plt.xlabel("Time [s]")
plt.ylabel("Torque [N m]")

plt.show()
```

---

## Notes on comparison with Simulink

The current Julia model is not expected to match the Simulink/Simscape model exactly yet.

Important possible differences are:

1. The Julia model currently applies a constant 25 Hz reference immediately.
2. The Simulink model may include a frequency ramp or rate limiter.
3. The Simulink model may include a series R/L element before the machine.
4. The Simscape machine block may include initialization details not yet reproduced here.
5. Delta/star parameter interpretation must be checked carefully.
6. The Julia model does not yet include the same sensors, filtering, or observer structure.

Therefore, the first validation target is qualitative:

```text
Does the machine accelerate toward approximately 750 rpm at 25 Hz?
```

After that, the next validation targets are:

```text
Does the current magnitude match?
Does the electromagnetic torque magnitude match?
Does the startup transient match?
Does the behavior under load torque match?
```

---

## Next development steps

Planned improvements include:

1. Add frequency ramp / rate limiter to the scalar V/f controller.
2. Add the same series R/L impedance as in the Simulink model.
3. Compare Julia and Simulink currents, torque, and speed using exported CSV data.
4. Add time-varying load torque profiles.
5. Add a flux and torque observer.
6. Add a winch/drum mechanical model.
7. Replace scalar control with field-oriented control.
8. Connect the model to AWES tether force or pumping-cycle profiles.
