# 160 kW induction-machine FOC F1 simulation

## Purpose

This test adds a 160 kW induction machine to the hybrid field-oriented control
(FOC) simulations. It validates bipolar speed tracking, opposing load-torque
steps, constant-flux operation, the transition to field weakening, voltage and
current limits, and optional load-torque feedforward.

The implementation consists of:

- `src/simulators/hybrid_foc_speed_f1_simulator.jl`: the general hybrid F1
  speed-control simulator and shared profile support;
- `src/simulators/hybrid_foc_speed_f1_160kw_simulator.jl`: the 160 kW machine
  parameters, bipolar test profiles, and specialized simulation entry point;
- `src/controls/FOC/outer_speed_flux_f1_discrete.jl`: the outer speed loop,
  constant-flux F1 reference, and field-weakening law;
- `scripts/run_foc_speed_f1_160kw.jl`: the executable test, metrics, plots, and
  figure export.

The specialized simulator and its two profile functions are included and
exported by `src/InductionMachineDrives.jl`:

```julia
simulate_foc_speed_f1_im_160kw
im_160kw_speed_reference
im_160kw_load_torque_profile
```

## Simulation structure

The simulation combines discrete control and estimation with a continuous
machine plant:

```text
speed reference
    -> F1 outer speed/flux controller
    -> dq current references
    -> discrete dq current controller
    -> inverse Park transformation
    -> stationary alpha-beta induction-machine plant (RK4)
    -> rotor-flux and torque observer
    -> measured speed and observed dq quantities
```

An optional Kalman estimator supplies a load-torque estimate to the outer-loop
feedforward path. The available modes are:

- `:none`: no load estimate;
- `:actual`: exact simulated load torque, useful for initial validation;
- `:kalman`: estimated load torque.

The plant, observer, controllers, and load estimator have independent parameter
scale factors. This allows robustness tests such as changing the actual rotor
resistance without changing the resistance assumed by the observer.

## 160 kW machine parameters

The default electrical parameters are derived from the 50 Hz equivalent-circuit
data used by the original initialization model. Reactances are converted to
inductances using

```math
L = \frac{X}{2\pi f_n}.
```

Important defaults include:

| Parameter | Default |
|---|---:|
| Rated power | 160 kW |
| Nominal frequency `fn` | 50 Hz |
| Pole pairs `p_pairs` | 3 |
| DC-link voltage `Vdc` | 605 V |
| Voltage limit `Vs_max` | `Vdc / sqrt(3)` |
| Nominal d-axis current `isd_nom` | `160sqrt(2)` A |
| Outer-loop current limit | `310sqrt(2)` A |
| Torque limit `Te_max` | 1540 N m |
| Inertia `J` | `130 / 13.1^2` kg m^2 |
| Sample time `Ts` | 100 microseconds |

The simulator returns both the parameter objects used by each subsystem and a
`nominal` tuple containing the derived inductances, leakage/transient
inductance, coupling factor, and rotor time constant.

## Constant flux and field weakening

F1 operation uses a constant nominal d-axis current below the field-weakening
base speed:

```math
i_{sd}^{*} = i_{sd,nom}.
```

When field weakening is enabled and the magnitude of the filtered or commanded
speed exceeds `wm_base_fw`, the desired d-axis current is reduced according to

```math
i_{sd}^{*} = \operatorname{clamp}\left(
    i_{sd,nom}\frac{\omega_{base}}{|\omega|},
    i_{sd,min},
    i_{sd,nom}
\right).
```

The d-axis reference is rate-limited by `id_dot_max` to avoid an abrupt flux
change. The controller exposes `field_weakening_active` so the transition can be
checked directly.

The default base speed is

```julia
wm_base_fw = 992.0 * 2pi / 60.0  # approximately 103.9 rad/s
```

The standard test commands ±50 rad/s. It therefore validates constant-flux F1
operation while leaving field weakening enabled and ready for higher speeds.
To exercise field weakening explicitly, use a speed above the base speed, for
example:

```julia
res_fw = simulate_foc_speed_f1_im_160kw(
    wm_ref_abs = 120.0,
    use_field_weakening = true,
)

maximum(res_fw.field_weakening_active) == 1.0
```

## Speed-reference profile

The default 13 s profile tests both rotation directions:

| Time [s] | Reference |
|---:|---:|
| 0-1 | 0 rad/s |
| 1-2 | Ramp to +50 rad/s |
| 2-5 | +50 rad/s |
| 5-6 | Ramp to 0 rad/s |
| 6-7 | 0 rad/s |
| 7-8 | Ramp to -50 rad/s |
| 8-11 | -50 rad/s |
| 11-12 | Ramp to 0 rad/s |
| 12-13 | 0 rad/s |

The profile magnitude and all transition times are keyword arguments of
`simulate_foc_speed_f1_im_160kw`.

## Load-torque profile and sign convention

The mechanical equation is

```math
J\frac{d\omega_m}{dt} = T_e + T_{load} - B\omega_m.
```

Consequently, the load profile uses negative torque during positive rotation
and positive torque during negative rotation. The default steps are:

| Time [s] | Load torque |
|---:|---:|
| 0-2.5 | 0 N m |
| 2.5-3.5 | -200 N m |
| 3.5-4.5 | -400 N m |
| 4.5-8.5 | 0 N m |
| 8.5-9.5 | +200 N m |
| 9.5-10.5 | +400 N m |
| 10.5-13 | 0 N m |

Thus, the disturbance opposes the commanded motion in both halves of the test.

## Running the test

From the project root, start Julia:

```bash
jl
```

At the Julia prompt, run:

```julia
include("scripts/run_foc_speed_f1_160kw.jl")
```

The script uses the exact simulated load torque for feedforward by default:

```julia
load_estimator = :actual
use_load_feedforward = true
```

Change `load_estimator` to `:none` to test feedback alone or to `:kalman` to
test the estimator-based feedforward path.

## Results and plots

The returned named tuple contains speed, torque, dq current and voltage,
observer, flux, three-phase, power, saturation, and field-weakening signals.
The run script reports:

- speed RMSE and peak absolute speed error;
- maximum reference and measured current magnitude;
- maximum applied and unsaturated voltage magnitude;
- current-, torque-, and q-current-saturation fractions;
- field-weakening active fraction;
- nominal rotor time constant and transient inductance.

MakieControlPlots generates three figures:

1. speed and torque;
2. dq currents, voltage utilization, and rotor flux;
3. current limits, power, and saturation flags.

They are saved under:

```text
results/im_160kw_f1/
```

with the filenames `speed_and_torque.png`, `currents_voltage_flux.png`, and
`current_power_saturation.png`.

## Suggested validation sequence

1. Run the default ±50 rad/s case and confirm that
   `field_weakening_active` remains zero.
2. Confirm that the measured speed follows both positive and negative
   references and that the applied load always opposes motion.
3. Compare `:actual`, `:none`, and `:kalman` load-estimator modes.
4. Raise `wm_ref_abs` above `wm_base_fw` and confirm that the d-axis current and
   flux references decrease while `field_weakening_active` is one.
5. Inspect `vs_mod`, `vs_mod_unsat`, and `ctrl_p.Vs_max` to assess voltage
   utilization in the field-weakening region.
6. Apply plant/observer scale-factor mismatches to evaluate robustness.
