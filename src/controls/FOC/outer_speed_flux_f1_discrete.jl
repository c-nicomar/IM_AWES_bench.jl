# ============================================================
# Discrete FOC outer loop: speed mode + T1 + F1
# ============================================================
#
# Simplified version of:
#
#   mode_control = 2  -> SPEED mode
#   mode_torque  = 1  -> T1, torque-to-iq using Kt_nom
#   mode_flux    = 1  -> F1, constant id reference
#
# It generates:
#   isd_ref, isq_ref
#
# to be consumed by current_controller_discrete.jl.
# ============================================================

"""
    OuterSpeedFluxF1State(; kwargs...)

Mutable state of the F1 constant-flux *speed* outer loop, advanced in place by
[`outer_speed_flux_f1_step!`](@ref) (`ant` is *anterior*, the previous value).

# Fields

- `ui_wm`: speed PI integrator state in N·m, corrected by back-calculation when
  the torque reference saturates.
- `wm_filt`: low-pass filtered mechanical speed measurement in rad/s.
- `wm_ref_ant`: previous ramped speed reference in rad/s. Its per-sample change
  also supplies the `J*dω/dt` feedforward term.
- `id_ref_ant`: previous ramped d-axis current reference in A, defaulting to the
  default `isd_nom` so a fresh state starts at nominal flux.

See also [`OuterSpeedFluxF1Params`](@ref), [`OuterSpeedFluxF1Output`](@ref).
"""
Base.@kwdef mutable struct OuterSpeedFluxF1State
    ui_wm::Float64 = 0.0
    wm_filt::Float64 = 0.0
    wm_ref_ant::Float64 = 0.0
    id_ref_ant::Float64 = 23.04579328
end

"""
    OuterSpeedFluxF1Params(; kwargs...)

Machine constants, tuning and limits of the F1 constant-flux speed outer loop,
consumed by [`outer_speed_flux_f1_step!`](@ref).

As in the inner loop, the PI gains are derived on every step rather than stored:
`Kp_wm = J/(ts_wm/3)`, `Ki_wm = Kp_wm/min(J/B, ts_dist_wm)` and
`Kaw_wm = 1/Kp_wm`.

# Fields

- `Ts`: sample time in s.
- `p`: pole pairs.
- `Lm`, `Lss`, `Lrr`: magnetizing, stator and rotor inductance in H. `Lss` is
  used only by the field-weakening voltage ellipse.
- `J`: total inertia in kg·m², motor plus load. Sets `Kp_wm` and the
  acceleration feedforward.
- `B`: viscous friction coefficient in N·m·s/rad, opposing motion in
  `J*dω/dt = Te + TL - B*ω`. Enters both the integral time constant and the
  friction feedforward.
- `isd_nom`: nominal d-axis current in A — the flux level F1 holds below base
  speed.
- `Is_max`: stator current magnitude limit in A.
- `isd_min`: lower bound in A on the d-axis reference, also the floor of field
  weakening.
- `Te_max`: torque reference magnitude limit in N·m.
- `wm_dot_max`: speed reference slew rate in rad/s². Set it very high (e.g.
  `1e6`) when a recorded profile should be played back unaltered.
- `id_dot_max`: d-axis current reference slew rate in A/s.
- `use_field_weakening`: when `true`, reduce flux above `wm_base_fw` instead of
  holding `isd_nom` at all speeds.
- `wm_base_fw`: mechanical base speed in rad/s above which field weakening
  starts. Compared internally in electrical terms, as `p*wm_base_fw`.
- `Vs_max`: stator voltage limit in V used by the field-weakening voltage
  ellipse. Keep it consistent with the inner controller's `Vs_max`.
- `tau_f_wm`: speed measurement filter time constant in s.
- `ts_wm`: specified closed-loop settling time of the speed loop in s.
- `ts_dist_wm`: disturbance-rejection time constant in s, capping the integral
  time constant when the mechanical time constant `J/B` is long.
- `use_load_feedforward`: when `true`, add `load_ff_sign * TL_est` to the
  feedforward torque. Requires a meaningful `TL_est`, e.g. from
  [`load_torque_kalman_step!`](@ref).
- `load_ff_sign`: sign of that term, `-1.0` by design. With
  `J*dω/dt = Te + TL - B*ω` a positive `TL` already accelerates the machine, so
  cancelling it requires `Te_ff = J*dωref/dt + B*ωref - TL_est`. Flipping this
  to `+1` doubles the disturbance instead of rejecting it.

See also [`OuterSpeedFluxF1State`](@ref), [`OuterSpeedFluxF1Output`](@ref).
"""
Base.@kwdef struct OuterSpeedFluxF1Params
    Ts::Float64 = 100e-6

    # Machine / nominal FOC parameters
    p::Float64 = 2.0
    Lm::Float64 = 40.84e-3
    Lss::Float64 = 45.12e-3
    Lrr::Float64 = 45.12e-3
    J::Float64 = 0.2685 + 0.1
    B::Float64 = 0.01298

    # F1 flux setting
    isd_nom::Float64 = 23.04579328

    # Limits
    Is_max::Float64 = 40.0*sqrt(2.0)
    isd_min::Float64 = 5.0
    Te_max::Float64 = 124.0419647
    wm_dot_max::Float64 = 100.0
    id_dot_max::Float64 = 600.0

    # Field weakening
    #
    # If enabled, the F1 flux reference is reduced above `wm_base_fw` using
    # both the inverse-speed limit and the steady-state voltage ellipse:
    #
    #   isd_ref = min(isd_nom, isd_speed_limit, isd_voltage_limit)
    #
    # with lower bound `isd_min`.
    use_field_weakening::Bool = false
    wm_base_fw::Float64 = 120.0
    Vs_max::Float64 = 310.0

    # Speed filter and PI design
    tau_f_wm::Float64 = 10e-3
    ts_wm::Float64 = 500e-3
    ts_dist_wm::Float64 = 3.0

    # Load torque feedforward flexibility.
    #
    # For now you probably want this false because no load torque estimator
    # is implemented yet. Later, TL_est can come from an estimator.
    use_load_feedforward::Bool = false

    # Sign convention for load feedforward.
    #
    # Mechanical convention:
    #   TL + Te = J*dω/dt + B*ω
    #   J*dω/dt = Te + TL - B*ω
    #
    # Therefore a positive TL already helps positive acceleration.
    # To compensate it in the speed loop:
    #
    #   Te_ff = J*dωref/dt + B*ωref - TL_est
    #
    # so the default feedforward sign is -1.
    load_ff_sign::Float64 = -1.0
end

"""
    OuterSpeedFluxF1Output(; kwargs...)

Result of one [`outer_speed_flux_f1_step!`](@ref) call, returned fresh each
sample. `isd_ref`/`isq_ref` drive the inner current controller; everything else
is diagnostic and is logged per sample by the speed simulators.

# Fields

- `isd_ref`, `isq_ref`: dq current references in A for
  [`current_controller_step!`](@ref).
- `Te_ref_out`: torque reference in N·m after clamping to `±Te_max`, i.e.
  `Te_PI + Te_ff` limited.
- `lambda_ref_out`: rotor flux reference in Wb, `Lm*isd_ref`. Equals
  `Lm*isd_nom` below base speed and drops in field weakening.
- `wm_ref_ramp`: speed reference in rad/s after `wm_dot_max` slew limiting — the
  reference the PI actually regulates against.
- `wm_filt`: filtered speed measurement in rad/s.
- `e_wm`: speed error `wm_ref_ramp - wm_filt` in rad/s.
- `e_Te`, `e_lambda`: always zero, kept for layout compatibility with the other
  outer loops.
- `sat_isd`, `sat_isq`, `sat_Te`: saturation flags, `+1` upper, `-1` lower, `0`
  unlimited.
- `T_load_est`: the `TL_est` argument passed through, whether or not feedforward
  used it.
- `Kt_nom`: torque constant in N·m/A at nominal flux.
- `isq_max_disp`: q-axis current in A still available inside the `Is_max` circle
  after the d-axis share.
- `Kt_ref`: torque constant in N·m/A at the *actual* flux reference. Differs from
  `Kt_nom` only under field weakening, and it is the one used for the
  torque-to-`isq` map.
- `isd_ref_no_fw`: the d-axis reference in A that would have been commanded
  without field weakening, i.e. `isd_nom`.
- `fw_speed`: `max(|wm_filt|, |wm_ref_ramp|)` in rad/s, the speed the
  field-weakening decision is based on. Using the larger of the two avoids
  weakening too late during fast reel-in/reel-out transients.
- `field_weakening_active`: `1` while the flux is being reduced, `0` otherwise.
- `Te_PI`: PI contribution to the torque reference in N·m.
- `Te_ff`: feedforward contribution in N·m,
  `load_ff_sign*TL_est + B*wm_ref_ramp + J*dωref/dt`.

See also [`OuterSpeedFluxF1Params`](@ref).
"""
Base.@kwdef struct OuterSpeedFluxF1Output
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

    Kt_nom::Float64 = 0.0
    isq_max_disp::Float64 = 0.0

    Kt_ref::Float64 = 0.0
    isd_ref_no_fw::Float64 = 0.0
    fw_speed::Float64 = 0.0
    field_weakening_active::Int = 0

    Te_PI::Float64 = 0.0
    Te_ff::Float64 = 0.0
end

function reset!(state::OuterSpeedFluxF1State, p::OuterSpeedFluxF1Params; wm_ref::Float64 = 0.0, wm_med::Float64 = 0.0)
    state.ui_wm = 0.0
    state.wm_filt = wm_med
    state.wm_ref_ant = wm_ref
    state.id_ref_ant = p.isd_nom
    return nothing
end

"""
    outer_speed_flux_f1_step!(state, p; wm_ref, wm_med, omega_e = nothing,
                              TL_est = 0.0, reset = false)

Advance the F1 constant-flux speed outer loop by one sample period `p.Ts` and
return the dq current references for the inner current controller.

This is the `mode_control = 2` (SPEED) + `mode_torque = 1` (T1) +
`mode_flux = 1` (F1) path of the MATLAB original, extended with optional
field weakening and optional load-torque feedforward. A PI on speed error plus a
model-based feedforward produces a torque reference, which is then mapped to
`isq` at the commanded flux level.

`state` is mutated in place; the result is a fresh
[`OuterSpeedFluxF1Output`](@ref).

# Arguments

- `state::OuterSpeedFluxF1State`: PI integrator, speed filter and the two ramp
  memories, mutated.
- `p::OuterSpeedFluxF1Params`: machine constants, tuning and limits.
- `wm_ref`: commanded mechanical speed in rad/s, before slew limiting. Signed —
  the loop tracks both directions.
- `wm_med`: measured mechanical speed in rad/s (`med` for *medido*).
- `omega_e`: synchronous *electrical* speed in rad/s, normally
  `RotorFluxObserverDiscreteOutput.omega_e`. Used only by field weakening. Pass
  `nothing` (the default) and the block falls back to the pole-pair-scaled
  mechanical speed `p*fw_speed`; a non-finite value takes the same fallback.
- `TL_est`: estimated load torque in N·m, positive when it accelerates toward
  positive speed. Only used when `p.use_load_feedforward` is `true`.
- `reset`: when `true`, zero the integrator, seed the speed filter with `wm_med`
  and the reference ramp with `wm_ref`, and restore nominal flux. Applied before
  anything else in the step.

# Steps

1. **Speed filtering and reference ramp.** One-pole filter with
   `alpha = Ts/(tau_f_wm + Ts)`; the reference is slew-limited at `wm_dot_max`.
2. **PI plus feedforward.**
   `Te_ref = Kp_wm*e_wm + ui_wm + (load_ff_sign*TL_est + B*wm_ref_ramp + J*dωref/dt)`,
   clamped to `±Te_max`. The integrator is then updated with
   `Ki_wm*e_wm*Ts` minus `Kaw_wm` times the clamped-away excess, so the loop
   does not wind up against the torque limit.
3. **Flux reference.** Constant `isd_nom`, unless `use_field_weakening` is `true`
   and the electrical speed exceeds `p*wm_base_fw`. Then `isd_ref` is the
   smallest of `isd_nom`, the inverse-speed limit `isd_nom*ω_base/ω`, and the
   steady-state voltage-ellipse limit built from `Vs_max`, floored at `isd_min`.
   The result is ramped at `id_dot_max`, since rotor flux cannot step.
4. **Torque-to-current map.** `isq_ref = Te_ref_out / Kt_ref` where `Kt_ref` uses
   the *achieved* flux `Lm*isd_ref`, not the nominal one — this is what keeps the
   torque correct while the field is weakened.
5. **Current-circle limiting, d-axis priority**, as in the torque loop.

Note that the mechanical convention `J*dω/dt = Te + TL - B*ω` is what fixes
`load_ff_sign = -1.0`. Changing that sign turns load feedforward from
compensation into amplification.

See also [`outer_speed_flux_mtpa_step!`](@ref) for the variable-flux
alternative, [`load_torque_kalman_step!`](@ref) for producing `TL_est`, and
[`simulate_foc_speed_f1_hybrid`](@ref) for a complete loop using this block.
"""
function outer_speed_flux_f1_step!(
    state::OuterSpeedFluxF1State,
    p::OuterSpeedFluxF1Params;
    wm_ref::Float64,
    wm_med::Float64,
    omega_e::Union{Nothing, Float64} = nothing,
    TL_est::Float64 = 0.0,
    reset::Bool = false,
)

    if reset
        reset!(state, p; wm_ref = wm_ref, wm_med = wm_med)
    end

    # ------------------------------------------------------------
    # Nominal constants
    # ------------------------------------------------------------

    k = p.Lm / p.Lrr
    lambda_rd_nom = p.Lm * p.isd_nom
    Kt_nom = 1.5 * p.p * k * lambda_rd_nom

    # ------------------------------------------------------------
    # Speed measurement filtering
    # ------------------------------------------------------------

    alpha_wm = p.Ts / (p.tau_f_wm + p.Ts)
    state.wm_filt = (1.0 - alpha_wm) * state.wm_filt + alpha_wm * wm_med
    wm_filt = state.wm_filt

    # ------------------------------------------------------------
    # Speed reference ramp
    # ------------------------------------------------------------

    delta_wm_max = p.wm_dot_max * p.Ts
    delta_wm = wm_ref - state.wm_ref_ant
    delta_wm = clamp(delta_wm, -delta_wm_max, delta_wm_max)

    wm_ref_ramp = state.wm_ref_ant + delta_wm
    state.wm_ref_ant = wm_ref_ramp

    # ------------------------------------------------------------
    # Speed PI design
    # Same structure as the MATLAB function.
    # ------------------------------------------------------------

    tau_cl_wm = p.ts_wm / 3.0
    B_safe = max(p.B, 1e-9)
    tau_m = p.J / B_safe
    tau_i_wm = min(tau_m, p.ts_dist_wm)

    Kp_wm = p.J / tau_cl_wm
    Ki_wm = Kp_wm / tau_i_wm
    Kaw_wm = 1.0 / Kp_wm

    e_wm = wm_ref_ramp - wm_filt

    up_wm = Kp_wm * e_wm
    Te_PI = up_wm + state.ui_wm

    dwm_ref_dt = delta_wm / p.Ts

    Te_ff_load = p.use_load_feedforward ? p.load_ff_sign * TL_est : 0.0
    Te_ff = Te_ff_load + p.B * wm_ref_ramp + p.J * dwm_ref_dt

    Te_ref_unsat = Te_PI + Te_ff

    if Te_ref_unsat > p.Te_max
        Te_ref_out = p.Te_max
        sat_Te = 1
    elseif Te_ref_unsat < -p.Te_max
        Te_ref_out = -p.Te_max
        sat_Te = -1
    else
        Te_ref_out = Te_ref_unsat
        sat_Te = 0
    end

    error_sat_wm = Te_ref_unsat - Te_ref_out
    state.ui_wm += Ki_wm * e_wm * p.Ts - Kaw_wm * error_sat_wm

    # ------------------------------------------------------------
    # F1 flux mode with optional speed-based field weakening
    # ------------------------------------------------------------

    isd_ref_no_fw = p.isd_nom

    # Use the larger of actual filtered speed and ramped reference speed.
    # This avoids waiting too long to weaken the field during high-speed
    # reel-in/reel-out transients.
    fw_speed = max(abs(wm_filt), abs(wm_ref_ramp))

    # Prefer the observer's synchronous electrical speed. Direct callers that
    # do not provide it retain the previous pole-pair-scaled mechanical-speed
    # approximation; non-finite observer values use the same safe fallback.
    omega_e_fw = if isnothing(omega_e) || !isfinite(omega_e)
        p.p * fw_speed
    else
        abs(omega_e)
    end
    omega_e_base_fw = p.p * p.wm_base_fw

    if p.use_field_weakening && omega_e_fw > omega_e_base_fw

        isd_speed_limit = p.isd_nom * omega_e_base_fw / omega_e_fw
        isd_preliminary = clamp(isd_speed_limit, p.isd_min, p.isd_nom)

        # Provisional q-axis current for the voltage-ellipse limit. The final
        # isq reference is still calculated below using the final isd_ref.
        lambda_preliminary = p.Lm * isd_preliminary
        Kt_preliminary = 1.5 * p.p * k * lambda_preliminary
        isq_preliminary_unsat = Te_ref_out / max(abs(Kt_preliminary), 1e-9)
        isq_preliminary_max = sqrt(max(p.Is_max^2 - isd_preliminary^2, 0.0))
        isq_preliminary = clamp(
            isq_preliminary_unsat,
            -isq_preliminary_max,
            isq_preliminary_max,
        )

        sigma = 1.0 - p.Lm^2 / (p.Lss * p.Lrr)
        voltage_radicand =
            (p.Vs_max / (omega_e_fw * p.Lss))^2 - sigma^2 * isq_preliminary^2
        isd_voltage_limit = sqrt(max(voltage_radicand, 0.0))

        isd_desired = clamp(
            min(p.isd_nom, isd_speed_limit, isd_voltage_limit),
            p.isd_min,
            p.isd_nom,
        )
        field_weakening_active = 1
    else
        isd_desired = p.isd_nom
        field_weakening_active = 0
    end

    # Keep the existing id ramp. This avoids abrupt flux/current steps.
    delta_id_max = p.id_dot_max * p.Ts
    delta_id = isd_desired - state.id_ref_ant
    delta_id = clamp(delta_id, -delta_id_max, delta_id_max)

    id_ref_ramp = state.id_ref_ant + delta_id
    state.id_ref_ant = id_ref_ramp

    if id_ref_ramp > p.Is_max
        isd_ref = p.Is_max
        sat_isd = 1
    elseif id_ref_ramp < p.isd_min
        isd_ref = p.isd_min
        sat_isd = -1
    else
        isd_ref = id_ref_ramp
        sat_isd = 0
    end

    # Rotor-flux reference associated with the final d-axis current reference.
    # Below base speed this equals lambda_rd_nom. In field weakening it is lower.
    lambda_ref_out = p.Lm * isd_ref

    # ------------------------------------------------------------
    # T1 torque mode: torque-to-iq using the weakened flux
    # ------------------------------------------------------------

    Kt_ref = 1.5 * p.p * k * lambda_ref_out
    Kt_ref_safe = max(abs(Kt_ref), 1e-9)

    isq_ref_unsat = Te_ref_out / Kt_ref_safe



    # ------------------------------------------------------------
    # q-axis current saturation with d-axis priority
    # ------------------------------------------------------------

    isq_max_disp = sqrt(max(p.Is_max^2 - isd_ref^2, 0.0))

    if isq_ref_unsat > isq_max_disp
        isq_ref = isq_max_disp
        sat_isq = 1
    elseif isq_ref_unsat < -isq_max_disp
        isq_ref = -isq_max_disp
        sat_isq = -1
    else
        isq_ref = isq_ref_unsat
        sat_isq = 0
    end

    return OuterSpeedFluxF1Output(
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

        Kt_nom = Kt_nom,
        isq_max_disp = isq_max_disp,

        Kt_ref = Kt_ref,
        isd_ref_no_fw = isd_ref_no_fw,
        fw_speed = fw_speed,
        field_weakening_active = field_weakening_active,

        Te_PI = Te_PI,
        Te_ff = Te_ff,
    )
end
