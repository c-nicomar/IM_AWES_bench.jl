# ============================================================
# Discrete FOC outer loop: speed mode + constrained MTPA
# ============================================================
#
# Specialized Julia counterpart of the MATLAB controller:
#
#   mode_control = SPEED
#   mode_flux    = F3 (MTPA with flux floor and torque reserve)
#   mode_torque  = direct torque-to-iq map based on
#                  Te = Kt_isd * isd * isq
#
# The block generates isd_ref and isq_ref for
# current_controller_discrete.jl.
#
# MTPA model:
#
#   Te = Kt_isd * isd * isq
#   Kt_isd = 1.5 * p * Lm^2 / Lrr
#
# Pure MTPA gives:
#
#   isd = |isq|
#   isd_mtpa = sqrt(abs(Te_ref) / Kt_isd)
#
# The practical reference also enforces:
#
#   isd >= lambda_rd_floor / Lm
#   isd >= Te_reserve / (1.5 * p * k * Lm * Is_max)
#   isd >= isd_min
#
# where k = Lm / Lrr.
# ============================================================

"""
    OuterSpeedFluxMTPAState(; kwargs...)

Mutable state of the constrained-MTPA speed outer loop, advanced in place by
[`outer_speed_flux_mtpa_step!`](@ref). Layout is identical to
[`OuterSpeedFluxF1State`](@ref) — the two controllers differ in how they choose
the flux level, not in what they remember.

# Fields

- `ui_wm`: speed PI integrator state in N·m.
- `wm_filt`: filtered mechanical speed in rad/s.
- `wm_ref_ant`: previous ramped speed reference in rad/s.
- `id_ref_ant`: previous ramped d-axis current reference in A. Unlike F1, this
  one keeps moving during normal operation, since MTPA retargets the flux
  whenever the torque demand changes.

See also [`OuterSpeedFluxMTPAParams`](@ref), [`OuterSpeedFluxMTPAOutput`](@ref).
"""
Base.@kwdef mutable struct OuterSpeedFluxMTPAState
    ui_wm::Float64 = 0.0
    wm_filt::Float64 = 0.0
    wm_ref_ant::Float64 = 0.0
    id_ref_ant::Float64 = 23.04579328
end

"""
    OuterSpeedFluxMTPAParams(; kwargs...)

Machine constants, tuning and limits of the constrained-MTPA speed outer loop,
consumed by [`outer_speed_flux_mtpa_step!`](@ref). The speed PI is designed
exactly as in [`OuterSpeedFluxF1Params`](@ref); the extra fields are the three
constraints that make plain MTPA usable on a winch.

# Fields

- `Ts`: sample time in s.
- `p`: pole pairs.
- `Lm`, `Lrr`: magnetizing and rotor inductance in H. They fix
  `Kt_isd = 1.5*p*Lm^2/Lrr`, the constant of the bilinear torque model
  `Te = Kt_isd * isd * isq`.
- `J`: total inertia in kg·m².
- `B`: viscous friction coefficient in N·m·s/rad.
- `isd_nom`: d-axis current in A used to initialise the ramp state on reset. In
  MTPA it is a starting point, not a setpoint.
- `Is_max`: stator current magnitude limit in A. Also caps the MTPA solution at
  `Is_max/sqrt(2)`, the point where `isd = isq` fills the current circle.
- `isd_min`: lower bound in A on the d-axis reference.
- `Te_max`: torque reference magnitude limit in N·m.
- `wm_dot_max`: speed reference slew rate in rad/s².
- `id_dot_max`: d-axis current reference slew rate in A/s.
- `lambda_rd_floor`: minimum rotor flux in Wb to hold regardless of torque
  demand. Pure MTPA collapses the flux near zero torque, which would leave the
  machine unable to respond; this is the floor that prevents it. Set to `0` to
  disable.
- `Te_reserve`: torque in N·m that must remain achievable within `Is_max` at any
  moment. Translated into a flux floor `Te_reserve/(1.5*p*k*Lm*Is_max)`, it buys
  fast response to a sudden load change at the cost of some efficiency. Set to
  `0` to disable.
- `tau_f_wm`: speed measurement filter time constant in s.
- `ts_wm`: specified closed-loop settling time of the speed loop in s.
- `ts_dist_wm`: disturbance-rejection time constant in s.
- `use_load_feedforward`: when `true`, add `load_ff_sign * TL_est` to the
  feedforward torque.
- `load_ff_sign`: `-1.0` by design, as required by `J*dω/dt = Te + TL - B*ω`.

See also [`OuterSpeedFluxMTPAState`](@ref), [`OuterSpeedFluxMTPAOutput`](@ref).
"""
Base.@kwdef struct OuterSpeedFluxMTPAParams
    Ts::Float64 = 100e-6

    # Machine / mechanical parameters
    p::Float64 = 2.0
    Lm::Float64 = 40.84e-3
    Lrr::Float64 = 45.12e-3
    J::Float64 = 0.2685 + 0.1
    B::Float64 = 0.01298

    # Initial / nominal flux-producing current
    isd_nom::Float64 = 23.04579328

    # Current and torque limits
    Is_max::Float64 = 40.0 * sqrt(2.0)
    isd_min::Float64 = 5.0
    Te_max::Float64 = 124.0419647

    # Reference slew-rate limits
    wm_dot_max::Float64 = 100.0
    id_dot_max::Float64 = 600.0

    # Constrained-MTPA settings
    lambda_rd_floor::Float64 = 0.35
    Te_reserve::Float64 = 45.0

    # Speed filter and PI design
    tau_f_wm::Float64 = 10e-3
    ts_wm::Float64 = 500e-3
    ts_dist_wm::Float64 = 3.0

    # Optional load-torque feedforward
    use_load_feedforward::Bool = false
    load_ff_sign::Float64 = -1.0
end

"""
    OuterSpeedFluxMTPAOutput(; kwargs...)

Result of one [`outer_speed_flux_mtpa_step!`](@ref) call, returned fresh each
sample. `isd_ref`/`isq_ref` drive the inner current controller; the diagnostics
below make it possible to see *which* constraint set the flux at any instant.

# Fields

- `isd_ref`, `isq_ref`: dq current references in A for
  [`current_controller_step!`](@ref).
- `Te_ref_out`: torque reference in N·m after clamping to `±Te_max`.
- `lambda_ref_out`: rotor flux reference in Wb, `Lm*isd_ref`. Unlike F1 this
  moves continuously with the torque demand.
- `wm_ref_ramp`: slew-limited speed reference in rad/s.
- `wm_filt`: filtered speed measurement in rad/s.
- `e_wm`: speed error in rad/s.
- `e_Te`, `e_lambda`: always zero, kept for layout compatibility.
- `sat_isd`, `sat_isq`, `sat_Te`: saturation flags, `+1` upper, `-1` lower, `0`
  unlimited.
- `T_load_est`: the `TL_est` argument passed through.
- `Kt_isd`: the bilinear torque coefficient `1.5*p*Lm^2/Lrr` in N·m/A².
- `isd_mtpa`: the unconstrained MTPA solution `sqrt(|Te_ref|/Kt_isd)` in A.
- `isd_floor`: flux-floor constraint `lambda_rd_floor/Lm` in A, or `0` when
  disabled.
- `isd_reserve`: torque-reserve constraint in A, or `0` when disabled.
- `isd_desired`: the applied target, `max` of the three above and `isd_min`,
  capped at `Is_max/sqrt(2)`. Comparing it against the three tells you which
  constraint is binding.
- `isq_max_disp`: q-axis current in A available inside the current circle.
- `Te_current_limited`: torque in N·m actually achievable with the commanded
  `isd_ref`/`isq_ref`, i.e. `Kt_isd*isd_ref*isq_ref`. Below `Te_ref_out`
  whenever the current circle bites.
- `torque_current_limited`: `1` when `isq` hit the circle limit, `0` otherwise.
- `Te_PI`: PI contribution in N·m.
- `Te_ff`: feedforward contribution in N·m.

See also [`OuterSpeedFluxMTPAParams`](@ref).
"""
Base.@kwdef struct OuterSpeedFluxMTPAOutput
    isd_ref::Float64 = 0.0
    isq_ref::Float64 = 0.0
    Te_ref_out::Float64 = 0.0
    lambda_ref_out::Float64 = 0.0

    wm_ref_ramp::Float64 = 0.0
    wm_filt::Float64 = 0.0
    e_wm::Float64 = 0.0
    e_Te::Float64 = 0.0
    e_lambda::Float64 = 0.0

    sat_isd::Int = 0
    sat_isq::Int = 0
    sat_Te::Int = 0
    T_load_est::Float64 = 0.0

    # Diagnostics
    Kt_isd::Float64 = 0.0
    isd_mtpa::Float64 = 0.0
    isd_floor::Float64 = 0.0
    isd_reserve::Float64 = 0.0
    isd_desired::Float64 = 0.0
    isq_max_disp::Float64 = 0.0
    Te_current_limited::Float64 = 0.0
    torque_current_limited::Int = 0
    Te_PI::Float64 = 0.0
    Te_ff::Float64 = 0.0
end

function reset!(
    state::OuterSpeedFluxMTPAState,
    p::OuterSpeedFluxMTPAParams;
    wm_ref::Float64 = 0.0,
    wm_med::Float64 = 0.0,
)
    state.ui_wm = 0.0
    state.wm_filt = wm_med
    state.wm_ref_ant = wm_ref
    state.id_ref_ant = p.isd_nom
    return nothing
end

"""
    outer_speed_flux_mtpa_step!(state, p; wm_ref, wm_med, TL_est = 0.0,
                                reset = false)

Advance the constrained-MTPA speed outer loop by one sample period `p.Ts` and
return the dq current references for the inner current controller.

Counterpart of [`outer_speed_flux_f1_step!`](@ref) with the same speed PI and
the same feedforward, but a different flux policy. F1 holds `isd` at `isd_nom`
whatever the torque; this block chooses `isd` per sample to minimise stator
current for the torque demanded — Maximum Torque Per Ampere — using the bilinear
model

```
Te = Kt_isd * isd * isq,    Kt_isd = 1.5 * p * Lm^2 / Lrr
```

whose optimum is `isd = |isq|`, i.e. `isd_mtpa = sqrt(|Te_ref|/Kt_isd)`. Pure
MTPA is not usable on its own — it starves the flux at low torque, so the
machine cannot respond to a sudden load — hence the three floors described
below. This is the `mode_flux = F3` path of the MATLAB original.

`state` is mutated in place; the result is a fresh
[`OuterSpeedFluxMTPAOutput`](@ref).

# Arguments

- `state::OuterSpeedFluxMTPAState`: PI integrator, speed filter and the two ramp
  memories, mutated.
- `p::OuterSpeedFluxMTPAParams`: machine constants, tuning, limits and the two
  constraint settings.
- `wm_ref`: commanded mechanical speed in rad/s, signed, before slew limiting.
- `wm_med`: measured mechanical speed in rad/s.
- `TL_est`: estimated load torque in N·m, positive when it accelerates toward
  positive speed. Used only when `p.use_load_feedforward` is `true`.
- `reset`: when `true`, zero the integrator, seed the speed filter with `wm_med`
  and the reference ramp with `wm_ref`, and set the flux ramp to `isd_nom`.

# Steps

1. **Speed filtering, reference ramp, PI plus feedforward** — identical to
   [`outer_speed_flux_f1_step!`](@ref), including the back-calculation
   anti-windup against `±Te_max`.
2. **Constrained flux choice.** `isd_desired` is the largest of the MTPA optimum
   `isd_mtpa`, the flux floor `lambda_rd_floor/Lm`, the torque reserve
   `Te_reserve/(1.5*p*k*Lm*Is_max)` and `isd_min`, then capped at
   `Is_max/sqrt(2)` — beyond that point a larger `isd` could not be matched by
   an equal `isq` inside the current circle. The result is ramped at
   `id_dot_max` because rotor flux cannot change instantaneously.
3. **Torque-to-current map.** `isq_ref = Te_ref_out / (Kt_isd * isd_ref)`, using
   the flux actually commanded this sample.
4. **Current-circle limiting, d-axis priority.** `isq_ref` is clamped to
   `sqrt(Is_max^2 - isd_ref^2)`; `Te_current_limited` then reports the torque
   that remains achievable.

There is deliberately no field weakening here: the flux is already speed-
independent and demand-driven. If you need voltage-limit-aware flux reduction,
use [`outer_speed_flux_f1_step!`](@ref) with `use_field_weakening = true`.

See also [`simulate_foc_speed_mtpa_hybrid`](@ref) for a complete loop using this
block.
"""
function outer_speed_flux_mtpa_step!(
    state::OuterSpeedFluxMTPAState,
    p::OuterSpeedFluxMTPAParams;
    wm_ref::Float64,
    wm_med::Float64,
    TL_est::Float64 = 0.0,
    reset::Bool = false,
)
    if reset
        reset!(state, p; wm_ref = wm_ref, wm_med = wm_med)
    end

    # ------------------------------------------------------------
    # Machine constants
    # ------------------------------------------------------------
    k = p.Lm / p.Lrr
    Kt_isd = 1.5 * p.p * p.Lm^2 / p.Lrr

    # ------------------------------------------------------------
    # Speed measurement filter
    # ------------------------------------------------------------
    alpha_wm = p.Ts / (p.tau_f_wm + p.Ts)
    state.wm_filt =
        (1.0 - alpha_wm) * state.wm_filt +
        alpha_wm * wm_med
    wm_filt = state.wm_filt

    # ------------------------------------------------------------
    # Speed-reference ramp
    # ------------------------------------------------------------
    delta_wm_max = p.wm_dot_max * p.Ts
    delta_wm = clamp(
        wm_ref - state.wm_ref_ant,
        -delta_wm_max,
        delta_wm_max,
    )

    wm_ref_ramp = state.wm_ref_ant + delta_wm
    state.wm_ref_ant = wm_ref_ramp

    # ------------------------------------------------------------
    # Speed PI, following the existing F1 controller structure
    # ------------------------------------------------------------
    tau_cl_wm = p.ts_wm / 3.0
    B_safe = max(p.B, 1e-9)
    tau_m = p.J / B_safe
    tau_i_wm = min(tau_m, p.ts_dist_wm)

    Kp_wm = p.J / tau_cl_wm
    Ki_wm = Kp_wm / tau_i_wm
    Kaw_wm = 1.0 / Kp_wm

    e_wm = wm_ref_ramp - wm_filt
    Te_PI = Kp_wm * e_wm + state.ui_wm

    dwm_ref_dt = delta_wm / p.Ts
    Te_ff_load =
        p.use_load_feedforward ?
        p.load_ff_sign * TL_est :
        0.0

    Te_ff =
        Te_ff_load +
        p.B * wm_ref_ramp +
        p.J * dwm_ref_dt

    Te_ref_unsat = Te_PI + Te_ff
    Te_ref_out = clamp(Te_ref_unsat, -p.Te_max, p.Te_max)

    sat_Te =
        Te_ref_unsat > p.Te_max ? 1 :
        Te_ref_unsat < -p.Te_max ? -1 :
        0

    error_sat_wm = Te_ref_unsat - Te_ref_out
    state.ui_wm +=
        Ki_wm * e_wm * p.Ts -
        Kaw_wm * error_sat_wm

    # ------------------------------------------------------------
    # Constrained MTPA d-axis current reference
    # ------------------------------------------------------------
    Kt_isd_safe = max(Kt_isd, 1e-9)
    isd_mtpa = sqrt(abs(Te_ref_out) / Kt_isd_safe)

    isd_floor =
        p.lambda_rd_floor > 0.0 ?
        p.lambda_rd_floor / max(p.Lm, 1e-9) :
        0.0

    reserve_den =
        1.5 * p.p * k * p.Lm * p.Is_max

    isd_reserve =
        p.Te_reserve > 0.0 ?
        p.Te_reserve / max(reserve_den, 1e-9) :
        0.0

    # Same upper bound used in the MATLAB F3 implementation.
    isd_max_mtpa = p.Is_max / sqrt(2.0)

    isd_desired = max(
        isd_mtpa,
        isd_floor,
        isd_reserve,
        p.isd_min,
    )
    isd_desired = min(isd_desired, isd_max_mtpa)

    # Rotor flux cannot change instantaneously, so retain the explicit
    # d-axis current slew-rate limit from the MATLAB implementation.
    delta_id_max = p.id_dot_max * p.Ts
    delta_id = clamp(
        isd_desired - state.id_ref_ant,
        -delta_id_max,
        delta_id_max,
    )

    id_ref_ramp = state.id_ref_ant + delta_id
    state.id_ref_ant = id_ref_ramp

    isd_ref = clamp(id_ref_ramp, p.isd_min, p.Is_max)

    sat_isd =
        id_ref_ramp > p.Is_max ? 1 :
        id_ref_ramp < p.isd_min ? -1 :
        0

    lambda_ref_out = p.Lm * isd_ref

    # ------------------------------------------------------------
    # Torque-to-iq map for MTPA
    # ------------------------------------------------------------
    isd_safe = max(isd_ref, p.isd_min, 1e-9)
    isq_ref_unsat = Te_ref_out / (Kt_isd_safe * isd_safe)

    # ------------------------------------------------------------
    # Current-circle saturation with d-axis priority
    # ------------------------------------------------------------
    isq_max_disp = sqrt(max(p.Is_max^2 - isd_ref^2, 0.0))
    isq_ref = clamp(
        isq_ref_unsat,
        -isq_max_disp,
        isq_max_disp,
    )

    sat_isq =
        isq_ref_unsat > isq_max_disp ? 1 :
        isq_ref_unsat < -isq_max_disp ? -1 :
        0

    Te_current_limited = Kt_isd * isd_ref * isq_ref
    torque_current_limited = sat_isq == 0 ? 0 : 1

    return OuterSpeedFluxMTPAOutput(
        isd_ref = isd_ref,
        isq_ref = isq_ref,
        Te_ref_out = Te_ref_out,
        lambda_ref_out = lambda_ref_out,
        wm_ref_ramp = wm_ref_ramp,
        wm_filt = wm_filt,
        e_wm = e_wm,
        e_Te = 0.0,
        e_lambda = 0.0,
        sat_isd = sat_isd,
        sat_isq = sat_isq,
        sat_Te = sat_Te,
        T_load_est = TL_est,
        Kt_isd = Kt_isd,
        isd_mtpa = isd_mtpa,
        isd_floor = isd_floor,
        isd_reserve = isd_reserve,
        isd_desired = isd_desired,
        isq_max_disp = isq_max_disp,
        Te_current_limited = Te_current_limited,
        torque_current_limited = torque_current_limited,
        Te_PI = Te_PI,
        Te_ff = Te_ff,
    )
end
