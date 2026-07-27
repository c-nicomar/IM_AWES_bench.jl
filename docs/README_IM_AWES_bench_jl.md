# IM_AWES_bench_jl

Julia simulation project for building and testing induction-machine control structures for AWES ground-station studies.

The project currently contains two modelling/simulation paths:

1. **ModelingToolkit symbolic models** for simple scalar V/f validation.
2. **Hybrid FOC simulations** with a continuous induction-machine plant and discrete controllers/observers, closer to the intended real implementation.

The current focus is the second path: a discrete FOC control structure with speed control, current control, rotor-flux/torque observation, optional load-torque estimation, and AWES profile playback from CSV.

---

## 1. Current folder structure

```text
IM_AWES_bench_jl/
├── docs/
│   └── README_IM_AWES_bench_jl.md
├── examples/
│   └── run_scalar_im.jl
├── profiles/
│   └── delta_kite_13_ms_profiles.csv
├── results/
│   ├── foc_current_hybrid_steps.csv
│   ├── foc_current_steps.csv
│   ├── foc_speed_f1_awes_profile.csv
│   ├── foc_speed_f1_ramp_load_estimator.csv
│   ├── foc_speed_f1_ramp_load_steps.csv
│   ├── foc_torque_f1_steps.csv
│   ├── scalar_frequency_steps_load_steps_with_observer.csv
│   ├── scalar_frequency_steps_load_steps.csv
│   ├── scalar_frequency_steps_results.csv
│   └── scalar_im_25Hz_results.csv
├── scripts/
│   ├── run_foc_current_hybrid_steps.jl
│   ├── run_foc_current_steps.jl
│   ├── run_foc_speed_f1_awes_profile.jl
│   ├── run_foc_speed_f1_ramp_load_estimator.jl
│   ├── run_foc_speed_f1_ramp_load_steps.jl
│   ├── run_foc_torque_f1_steps.jl
│   ├── run_scalar_frequency_steps.jl
│   └── run_scalar_frequency_steps_load_steps.jl
└── src/
    ├── controls/
    │   ├── scalar_vf_control.jl
    │   └── FOC/
    │       ├── current_controller.jl
    │       ├── current_controller_discrete.jl
    │       ├── outer_speed_flux_f1_discrete.jl
    │       └── outer_torque_flux_f1_discrete.jl
    ├── estimators/
    │   ├── load_torque_kalman_discrete.jl
    │   ├── rotor_flux_observer_discrete.jl
    │   └── rotor_flux_observer.jl
    ├── plants/
    │   └── induction_machine_alpha_beta.jl
    ├── profiles/
    │   ├── frequency_profiles.jl
    │   └── load_profiles.jl
    ├── simulators/
    │   ├── hybrid_foc_current_simulator.jl
    │   ├── hybrid_foc_speed_f1_simulator.jl
    │   └── hybrid_foc_torque_f1_simulator.jl
    ├── systems/
    │   ├── foc_current_im_system.jl
    │   └── scalar_im_system.jl
    └── IM_AWES_bench_jl.jl
```

---

## 2. Main project entry point

The main module file is:

```text
src/IM_AWES_bench_jl.jl
```

This file includes all submodules/files and exports the main simulation functions.

Typical include structure:

```julia
include("profiles/frequency_profiles.jl")
include("profiles/load_profiles.jl")

include("controls/scalar_vf_control.jl")
include("controls/FOC/current_controller.jl")
include("controls/FOC/current_controller_discrete.jl")
include("controls/FOC/outer_torque_flux_f1_discrete.jl")
include("controls/FOC/outer_speed_flux_f1_discrete.jl")

include("plants/induction_machine_alpha_beta.jl")

include("estimators/rotor_flux_observer.jl")
include("estimators/rotor_flux_observer_discrete.jl")
include("estimators/load_torque_kalman_discrete.jl")

include("systems/scalar_im_system.jl")
include("systems/foc_current_im_system.jl")

include("simulators/hybrid_foc_current_simulator.jl")
include("simulators/hybrid_foc_torque_f1_simulator.jl")
include("simulators/hybrid_foc_speed_f1_simulator.jl")
```

The most relevant exported simulation functions are:

```julia
simulate_scalar_im(...)   # needs `using ModelingToolkit, OrdinaryDiffEq` (package extension)
simulate_foc_current_hybrid(...)
simulate_foc_torque_f1_hybrid(...)
simulate_foc_speed_f1_hybrid(...)
```

---

## 3. Mechanical sign convention

The project now uses the following mechanical equation:

```text
TL + Te = J*dω/dt + B*ω
```

or equivalently:

```text
J*dω/dt = Te + TL - B*ω
```

Meaning:

```text
Te > 0      electromagnetic torque accelerates toward positive speed
TL > 0      external/load torque also pulls toward positive speed
Bω          viscous friction opposes motion
```

This is important for all plant, estimator, and feedforward logic.

For the speed-loop feedforward:

```text
Te_ff = J*dωref/dt + B*ωref - TL_est
```

Therefore, in speed-loop run scripts:

```julia
load_ff_sign = -1.0
```

For AWES profile playback:

```julia
profile_torque_sign = 1.0
```

should be used if the CSV torque profile already follows the convention:

```text
positive torque_ref_Nm pulls toward positive speed
```

Use:

```julia
profile_torque_sign = -1.0
```

only if the CSV torque has the opposite sign convention.

---

## 4. Plant model

### File

```text
src/plants/induction_machine_alpha_beta.jl
```

This is the symbolic/MTK induction-machine model in stationary alpha-beta coordinates.

It defines the induction-machine equations:

```text
stator voltage equations
rotor squirrel-cage equations
flux-current relations
electromagnetic torque
mechanical dynamics
```

The mechanical equation should use the project convention:

```julia
J * D(ωm) ~ Te + Tload_cmd - B * ωm
```

### Hybrid plant

The hybrid FOC simulations use a hand-written plant derivative function in:

```text
src/simulators/hybrid_foc_current_simulator.jl
```

The corresponding mechanical derivative should be:

```julia
dωm = (Te + Tload - p.B * x.ωm) / p.J
```

The speed and torque FOC hybrid simulators reuse this plant implementation.

---

## 5. Scalar V/f model path

This path is useful for basic plant validation and comparison against Simulink.

### Key files

```text
src/controls/scalar_vf_control.jl
src/profiles/frequency_profiles.jl
src/profiles/load_profiles.jl
src/systems/scalar_im_system.jl
```

### Main scripts

```text
scripts/run_scalar_frequency_steps.jl
scripts/run_scalar_frequency_steps_load_steps.jl
examples/run_scalar_im.jl
```

### Concept

The scalar system builds a symbolic MTK system:

```text
frequency profile -> scalar V/f control -> alpha-beta induction machine plant
load profile -----------------------------> plant mechanical input
```

The scalar V/f controller generates alpha-beta stator voltages directly from a frequency command.

---

## 6. Hybrid FOC simulation path

The hybrid FOC path is the preferred architecture for controller development.

It uses:

```text
continuous induction-machine plant
+
discrete observers/controllers
+
fixed-step simulation loop
```

This is closer to the real implementation, where controllers run at a fixed sampling time.

### Core idea

At every sample time:

```text
1. Read plant currents and mechanical speed/angle
2. Update rotor-flux/torque observer
3. Optionally update load-torque estimator
4. Update outer loop, if present
5. Update inner current controller
6. Convert dq voltage command to alpha-beta
7. Integrate the plant over one sample period
8. Store results
```

---

## 7. Discrete inner current controller

### File

```text
src/controls/FOC/current_controller_discrete.jl
```

### Purpose

This is the discrete FOC current controller. It receives:

```text
isd_ref, isq_ref
isd_meas, isq_meas
omega_e
lambda_rd
```

and outputs:

```text
vsd, vsq
```

It includes:

```text
current reference limiting
optional measurement filtering
PI control
optional feedforward decoupling
optional voltage saturation
optional anti-windup
```

### Main types/functions

```julia
CurrentControllerDiscreteState
CurrentControllerDiscreteParams
CurrentControllerDiscreteOutput
current_controller_step!(...)
```

---

## 8. Rotor-flux and torque observer

### File

```text
src/estimators/rotor_flux_observer_discrete.jl
```

### Purpose

This observer estimates rotor flux and torque from measured alpha-beta currents and mechanical rotor angle.

It outputs:

```text
lambda_rd_e
lambda_rq_e
theta_e
i_sd_e
i_sq_e
Te_raw
omega_e
```

The current controller uses the observer outputs as its synchronous-frame measurements.

### Main types/functions

```julia
RotorFluxObserverDiscreteState
RotorFluxObserverDiscreteParams
RotorFluxObserverDiscreteOutput
rotor_flux_observer_step!(...)
```

---

## 9. Load-torque Kalman estimator

### File

```text
src/estimators/load_torque_kalman_discrete.jl
```

### Purpose

This estimator estimates the external torque `TL` from speed and electromagnetic torque.

It is based on the mechanical model:

```text
J*dω/dt = Te + TL - B*ω
```

Discrete model:

```text
ω[k+1] = (1 - B*Ts/J)*ω[k] + (Ts/J)*TL[k] + (Ts/J)*Te[k]
TL[k+1] = TL[k]
```

Therefore, the estimator should use:

```julia
A12 = Ts / J
```

not `-Ts/J`.

### Main types/functions

```julia
LoadTorqueKalmanState
LoadTorqueKalmanParams
LoadTorqueKalmanOutput
load_torque_kalman_step!(...)
```

For AWES profiles, where torque may be signed, it is usually safer to use:

```julia
TL_kalman_limit_positive = false
```

---

## 10. Outer torque loop, F1 constant flux

### File

```text
src/controls/FOC/outer_torque_flux_f1_discrete.jl
```

### Purpose

Simplified outer-loop case equivalent to:

```text
mode_control = 1   torque control
mode_torque  = 1   simple torque-to-iq conversion
mode_flux    = 1   constant id reference
```

It converts:

```text
Te_ref -> isd_ref, isq_ref
```

with:

```text
isd_ref = isd_nom
isq_ref = Te_ref / Kt_nom
```

and current-circle limitation.

### Used by

```text
src/simulators/hybrid_foc_torque_f1_simulator.jl
scripts/run_foc_torque_f1_steps.jl
```

---

## 11. Outer speed loop, F1 constant flux

### File

```text
src/controls/FOC/outer_speed_flux_f1_discrete.jl
```

### Purpose

Simplified speed-loop case equivalent to:

```text
mode_control = 2   speed control
mode_torque  = 1   simple torque-to-iq conversion
mode_flux    = 1   constant id reference
```

It converts:

```text
speed_ref -> Te_ref -> isd_ref, isq_ref
```

using a discrete speed PI and optional load-torque feedforward.

With the current mechanical convention:

```text
J*dω/dt = Te + TL - B*ω
```

load feedforward should use:

```julia
load_ff_sign = -1.0
```

so that:

```text
Te_ff = J*dωref/dt + B*ωref - TL_est
```

### Used by

```text
src/simulators/hybrid_foc_speed_f1_simulator.jl
scripts/run_foc_speed_f1_ramp_load_steps.jl
scripts/run_foc_speed_f1_ramp_load_estimator.jl
scripts/run_foc_speed_f1_awes_profile.jl
```

---

## 12. Hybrid current-loop simulator

### File

```text
src/simulators/hybrid_foc_current_simulator.jl
```

### Purpose

Tests only the inner current controller:

```text
isd_ref, isq_ref -> current controller -> voltage command -> plant
```

It is useful for debugging current tracking before adding torque or speed outer loops.

### Main script

```text
scripts/run_foc_current_hybrid_steps.jl
```

### Typical reference sequence

```text
0–1 s      id = 0 A, iq = 0 A
1–3 s      id = 5 A, iq = 0 A
3–6 s      id = 5 A, iq = 5 A
6–9 s      id = 5 A, iq = -5 A
9–12 s     id = 5 A, iq = 0 A
```

---

## 13. Hybrid torque-loop simulator

### File

```text
src/simulators/hybrid_foc_torque_f1_simulator.jl
```

### Purpose

Tests the torque outer loop plus inner current loop:

```text
Te_ref -> outer torque F1 -> isd_ref, isq_ref -> current controller -> plant
```

### Main script

```text
scripts/run_foc_torque_f1_steps.jl
```

### Typical reference sequence

```text
0–2 s       Te_ref = 0 Nm
2–6 s       Te_ref = +20 Nm
6–10 s      Te_ref = -20 Nm
10–12 s     Te_ref = 0 Nm
```

---

## 14. Hybrid speed-loop simulator

### File

```text
src/simulators/hybrid_foc_speed_f1_simulator.jl
```

### Purpose

Tests speed outer-loop FOC with constant-flux F1 mode:

```text
speed_ref -> speed PI -> Te_ref -> isd_ref, isq_ref -> current controller -> plant
```

It supports:

```text
internal ramp speed references
internal load-step profiles
external CSV speed/load profiles
optional load-torque Kalman estimator
optional load-torque feedforward
abc reconstruction
electrical/mechanical power calculation
```

### Main scripts

```text
scripts/run_foc_speed_f1_ramp_load_steps.jl
scripts/run_foc_speed_f1_ramp_load_estimator.jl
scripts/run_foc_speed_f1_awes_profile.jl
```

---

## 15. AWES CSV profile playback

### Input file

```text
profiles/delta_kite_13_ms_profiles.csv
```

Expected columns:

```text
t_s
speed_ref_rpm
speed_ref_rad_s
torque_ref_Nm
```

The current AWES run script uses:

```text
speed_ref_rpm      -> speed reference profile
torque_ref_Nm      -> external torque profile
```

The simulator linearly interpolates between profile points at each simulation time, similar to Simulink timeseries behavior.

### Main script

```text
scripts/run_foc_speed_f1_awes_profile.jl
```

### Important settings

```julia
speed_reference_source = :profile
load_source = :profile
profile_torque_sign = 1.0
wm_dot_max = 1e6
load_estimator = :kalman
use_load_feedforward = true
load_ff_sign = -1.0
TL_kalman_limit_positive = false
```

`wm_dot_max = 1e6` effectively disables internal speed-reference ramp limiting, so the speed reference used by the PI follows the CSV profile closely.

---

## 16. Power and abc current reconstruction

The hybrid speed simulator reconstructs abc quantities from alpha-beta quantities assuming zero-sequence is zero:

```julia
ia = isα
ib = -0.5 * isα + sqrt(3)/2 * isβ
ic = -0.5 * isα - sqrt(3)/2 * isβ

va = vsα
vb = -0.5 * vsα + sqrt(3)/2 * vsβ
vc = -0.5 * vsα - sqrt(3)/2 * vsβ
```

Instantaneous stator electrical power:

```julia
Pelec = va*ia + vb*ib + vc*ic
```

Mechanical electromagnetic power:

```julia
Pmech = Te * ωm
```

External torque power:

```julia
Pload = TL * ωm
```

Friction loss:

```julia
Pfric = B * ωm^2
```

With the current sign convention:

```text
Pload > 0  external torque injects mechanical power into the shaft
Pload < 0  external torque extracts mechanical power from the shaft
```

---

## 17. Running simulations

From the project root:

```bash
julia --project=. scripts/run_foc_current_hybrid_steps.jl
julia --project=. scripts/run_foc_torque_f1_steps.jl
julia --project=. scripts/run_foc_speed_f1_ramp_load_steps.jl
julia --project=. scripts/run_foc_speed_f1_ramp_load_estimator.jl
julia --project=. scripts/run_foc_speed_f1_awes_profile.jl
```

For the AWES profile case:

```bash
julia --project=. scripts/run_foc_speed_f1_awes_profile.jl
```

The result CSV is saved in:

```text
results/foc_speed_f1_awes_profile.csv
```

---

## 18. Output results

Simulation outputs are saved in:

```text
results/
```

The AWES profile script saves:

```text
results/foc_speed_f1_awes_profile.csv
```

This CSV includes:

```text
speed reference and measured speed
torque reference, plant torque, observed torque, AWES torque, estimated load torque
dq currents
alpha-beta currents and voltages
reconstructed abc currents and voltages
electrical/mechanical/load/friction power
rotor flux and angle
saturation/debug signals
```

---

## 19. Recommended validation workflow

### Step 1: Validate current loop

```bash
julia --project=. scripts/run_foc_current_hybrid_steps.jl
```

Check:

```text
id tracking
iq tracking
positive iq gives positive torque
negative iq gives negative torque
```

### Step 2: Validate torque loop

```bash
julia --project=. scripts/run_foc_torque_f1_steps.jl
```

Check:

```text
Te_ref -> iq_ref conversion
torque response
speed acceleration/deceleration
```

### Step 3: Validate speed loop without load estimator

```julia
load_estimator = :none
use_load_feedforward = false
```

### Step 4: Validate load estimator only

```julia
load_estimator = :kalman
use_load_feedforward = false
```

Check whether `TL_est` tracks `Tload`.

### Step 5: Validate load feedforward

```julia
load_estimator = :kalman
use_load_feedforward = true
load_ff_sign = -1.0
```

Check whether speed error improves when the load torque changes.

### Step 6: Run AWES profile

```bash
julia --project=. scripts/run_foc_speed_f1_awes_profile.jl
```

Check:

```text
speed profile tracking
AWES torque sign
estimated load torque
abc currents
Pelec, Pmech, Pload
```

---

## 20. Common issues

### Wrong load-torque sign

If enabling load feedforward makes the speed tracking worse, check:

```julia
load_ff_sign = -1.0
profile_torque_sign = 1.0
```

and verify the mechanical convention:

```text
J*dω/dt = Te + TL - Bω
```

### AWES profile does not match input

Check:

```julia
speed_reference_source = :profile
load_source = :profile
wm_dot_max = 1e6
```

Remember:

```text
res.wm_ref       = raw CSV speed reference after interpolation
res.wm_ref_ramp  = ramp-limited reference used internally by the speed PI
```

### Load estimator cannot estimate negative torque

Set:

```julia
TL_kalman_limit_positive = false
```

---

## 21. Current development status

Implemented:

```text
scalar V/f MTK system
alpha-beta induction-machine plant
hybrid FOC current-loop simulation
discrete current controller
discrete rotor-flux/torque observer
discrete load-torque Kalman estimator
outer torque loop, T1 + F1
outer speed loop, T1 + F1
AWES speed/torque profile playback from CSV
abc current/voltage reconstruction
stator electrical power and mechanical power calculation
```

Not yet implemented:

```text
advanced FOC modes from the full MATLAB controller
MTPA / loss minimization modes
field-weakening
full voltage/current constraint management beyond the current controller level
higher-level AWES cycle energy metrics
comparison automation across controller options
parameter sweep / robustness batch scripts
```

