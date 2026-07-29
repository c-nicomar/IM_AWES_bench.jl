# ============================================================
# Discrete FOC current controller for induction machine
# ============================================================
#
# This implements the logic of the MATLAB current controller in a
# sample-by-sample way.
#
# Inputs:
#   isd_ref, isq_ref
#   isd_med, isq_med
#   omega_e
#   lambda_rd
#   Vs_max
#   Is_max
#
# Outputs:
#   vsd, vsq
#   limited current references
#   saturation flag
#   current magnitude
#
# The state variables ui_d, ui_q, isd_filt, isq_filt are stored in
# CurrentControllerDiscreteState.
# ============================================================

"""
    CurrentControllerDiscreteState(; kwargs...)

Mutable state carried by the discrete FOC inner current loop between samples,
advanced in place by [`current_controller_step!`](@ref). All fields default to
zero, which is the correct start for a machine at standstill; a non-zero start
is better handled by passing `reset = true` on the first step, which seeds the
filters from the present measurements.

# Fields

- `ui_d`, `ui_q`: d- and q-axis PI integrator states in V. Updated by
  `Ki * err * Ts` each sample, minus the anti-windup back-calculation term when
  the voltage command saturates.
- `isd_filt`, `isq_filt`: one-pole low-pass filtered dq current measurements in
  A. Bypassed (set equal to the raw measurement) when `use_filter` is `false`.

See also [`CurrentControllerDiscreteParams`](@ref),
[`CurrentControllerDiscreteOutput`](@ref).
"""
Base.@kwdef mutable struct CurrentControllerDiscreteState
    ui_d::Float64 = 0.0
    ui_q::Float64 = 0.0
    isd_filt::Float64 = 0.0
    isq_filt::Float64 = 0.0
end

"""
    CurrentControllerDiscreteParams(; kwargs...)

Immutable tuning and machine constants of the discrete FOC inner current loop,
consumed by [`current_controller_step!`](@ref). Defaults describe the small
bench machine; the 160 kW case passes its own values.

The PI gains are not stored — they are derived from these parameters on every
step as `tau_cl = ts_spec/3`, `Kp = sigma_Lss/tau_cl`, `Ki = Rs/tau_cl` and
`Kaw = 1/Kp`, so changing `ts_spec` between steps retunes the loop immediately.

# Fields

- `Rs`: stator resistance in Ω, sets the integral gain.
- `sigma_Lss`: transient inductance `Lss - Lm^2/Lrr` in H, sets the proportional
  gain and the cross-coupling feedforward.
- `k`: rotor coupling coefficient `Lm/Lrr`, dimensionless, scales the back-EMF
  feedforward.
- `Ts`: controller sample time in s.
- `ts_spec`: specified closed-loop settling time of the current loop in s.
- `tau_f`: time constant in s of the optional current measurement filter.
- `Vs_max`: stator voltage magnitude limit in V. The command is scaled, not
  clipped per axis, so its direction survives saturation.
- `Is_max`: stator current magnitude limit in A, applied to the *references* with
  d-axis priority.
- `use_filter`, `use_feedforward`, `use_saturation`, `use_antiwindup`:
  development flags for switching the corresponding stage off. All default to
  `true`, which is what every simulator in this package uses.

See also [`CurrentControllerDiscreteState`](@ref),
[`CurrentControllerDiscreteOutput`](@ref).
"""
Base.@kwdef struct CurrentControllerDiscreteParams
    Rs::Float64 = 0.21946

    # Transient inductance sigma*Lss.
    # For this machine:
    # sigma_Lss = Lss - Lm^2/Lrr
    sigma_Lss::Float64 = 0.00815

    # Coupling coefficient k = Lm/Lrr
    k::Float64 = 0.9051

    Ts::Float64 = 100e-6

    # Current-loop tuning
    ts_spec::Float64 = 2e-3

    # Optional measurement filter
    tau_f::Float64 = 50e-6

    # Limits
    Vs_max::Float64 = 310.0
    Is_max::Float64 = 40.0

    # Debug/development flags
    use_filter::Bool = true
    use_feedforward::Bool = true
    use_saturation::Bool = true
    use_antiwindup::Bool = true
end

"""
    CurrentControllerDiscreteOutput(; kwargs...)

Result of one [`current_controller_step!`](@ref) call. A fresh instance is
returned each sample; nothing here is carried over, the loop's memory lives in
[`CurrentControllerDiscreteState`](@ref).

Only `vsd`/`vsq` drive the plant. Everything else is diagnostic and is what the
simulators log per sample.

# Fields

- `vsd`, `vsq`: the dq stator voltage command in V, after feedforward and
  saturation. This is the block's actual output, rotated to alpha-beta by the
  caller.
- `isd_ref_lim`, `isq_ref_lim`: current references in A after limiting to the
  `Is_max` circle with d-axis priority.
- `isd_filt`, `isq_filt`: the filtered measurements in A actually used this
  sample.
- `err_d`, `err_q`: the regulated current errors in A.
- `vsd_PI`, `vsq_PI`: PI contribution in V, proportional plus integrator.
- `vsd_ff`, `vsq_ff`: decoupling feedforward in V. `vsq_ff` includes the back-EMF
  term `omega_e*k*lambda_rd`. Both are zero when `use_feedforward` is `false`.
- `vsd_unsat`, `vsq_unsat`: the command in V before saturation, i.e. PI plus
  feedforward.
- `vs_mod_unsat`: magnitude of that unsaturated command in V. Compare against
  `Vs_max` to see how much voltage headroom is left.
- `saturado`: `true` when the voltage limit was hit this sample (*saturado* is
  Spanish for saturated, kept from the MATLAB original).
- `is_mod`: magnitude of the filtered current vector in A.

See also [`CurrentControllerDiscreteParams`](@ref).
"""
Base.@kwdef struct CurrentControllerDiscreteOutput
    vsd::Float64 = 0.0
    vsq::Float64 = 0.0

    isd_ref_lim::Float64 = 0.0
    isq_ref_lim::Float64 = 0.0

    isd_filt::Float64 = 0.0
    isq_filt::Float64 = 0.0

    err_d::Float64 = 0.0
    err_q::Float64 = 0.0

    vsd_PI::Float64 = 0.0
    vsq_PI::Float64 = 0.0

    vsd_ff::Float64 = 0.0
    vsq_ff::Float64 = 0.0

    vsd_unsat::Float64 = 0.0
    vsq_unsat::Float64 = 0.0

    vs_mod_unsat::Float64 = 0.0
    saturado::Bool = false

    is_mod::Float64 = 0.0
end

function reset!(state::CurrentControllerDiscreteState)
    state.ui_d = 0.0
    state.ui_q = 0.0
    state.isd_filt = 0.0
    state.isq_filt = 0.0
    return nothing
end

"""
    current_controller_step!(state, p; isd_ref, isq_ref, isd_med, isq_med,
                             omega_e, lambda_rd, reset = false)

Advance the discrete FOC inner current loop by one sample period `p.Ts`.

This is the innermost block of the hybrid loop: it turns dq current references
into the dq stator voltage command that is then rotated to alpha-beta and
applied to the plant. It is called once per fixed-step iteration, after the
outer torque or speed loop has produced `isd_ref`/`isq_ref`.

`state` is mutated in place — the two integrator states `ui_d`/`ui_q` and the
two filtered measurements `isd_filt`/`isq_filt`. The result is returned as a
fresh `CurrentControllerDiscreteOutput`.

# Arguments

- `state::CurrentControllerDiscreteState`: integrator and filter state, mutated.
- `p::CurrentControllerDiscreteParams`: machine constants, tuning, limits and
  the four feature flags.
- `isd_ref`, `isq_ref`: dq current references in A, before limiting.
- `isd_med`, `isq_med`: measured dq currents in A (`med` for *medido*).
- `omega_e`: electrical angular frequency in rad/s, used by the decoupling
  feedforward only.
- `lambda_rd`: d-axis rotor flux linkage in Wb, normally from the rotor-flux
  observer.
- `reset`: when `true`, zero both integrators and seed the filter states with
  the present measurements, giving a bumpless start. Applied before anything
  else in the step.

# Steps

1. **PI gains.** Recomputed every call from `p`: `tau_cl = ts_spec/3`,
   `Kp = sigma_Lss/tau_cl`, `Ki = Rs/tau_cl`, and the back-calculation gain
   `Kaw = 1/Kp`. The integrator accumulates `Ki * err * Ts`, so `Ki` carries
   units of V/(A·s).
2. **Measurement filter.** With `use_filter`, a one-pole low pass with
   `alpha = Ts/(tau_f + Ts)`; otherwise the raw measurements pass through.
3. **Reference limiting, d-axis priority.** The flux-producing axis wins: if
   `|isd_ref| > Is_max` it is clamped and `isq_ref_lim` is forced to zero.
   Otherwise `isq_ref` is limited to `sqrt(Is_max^2 - isd_ref_lim^2)`, so the
   commanded current vector stays inside the `Is_max` circle.
4. **PI on the errors**, proportional part plus stored integrator.
5. **Feedforward decoupling**, with `use_feedforward`:
   `vsd_ff = -omega_e*sigma_Lss*isq_filt` and
   `vsq_ff = omega_e*sigma_Lss*isd_filt + omega_e*k*lambda_rd`, the second term
   being the back-EMF.
6. **Voltage saturation**, with `use_saturation`: if the unsaturated command
   exceeds `Vs_max` in magnitude, both components are scaled by the same factor,
   preserving direction rather than clipping per axis, and `saturado` is set.
7. **Anti-windup**, with `use_antiwindup`: when saturated, the integrator is
   corrected by `Kaw` times the difference between the unsaturated and
   saturated *PI* parts, feedforward excluded.

The four flags exist for development and bring-up; all default to `true`, which
is the configuration the simulators use.

See also [`CurrentControllerDiscreteState`](@ref),
[`CurrentControllerDiscreteParams`](@ref) and
[`CurrentControllerDiscreteOutput`](@ref).
"""
function current_controller_step!(
    state::CurrentControllerDiscreteState,
    p::CurrentControllerDiscreteParams;
    isd_ref::Float64,
    isq_ref::Float64,
    isd_med::Float64,
    isq_med::Float64,
    omega_e::Float64,
    lambda_rd::Float64,
    reset::Bool = false,
)

    if reset
        state.ui_d = 0.0
        state.ui_q = 0.0
        state.isd_filt = isd_med
        state.isq_filt = isq_med
    end

    Ts = p.Ts

    # ------------------------------------------------------------
    # PI design
    # ------------------------------------------------------------

    tau_cl = p.ts_spec / 3.0

    Kp = p.sigma_Lss / tau_cl
    Ki = p.Rs / tau_cl
    Kaw = 1.0 / Kp

    # ------------------------------------------------------------
    # Measurement filtering
    # ------------------------------------------------------------

    if p.use_filter
        alpha = Ts / (p.tau_f + Ts)
        state.isd_filt = (1.0 - alpha) * state.isd_filt + alpha * isd_med
        state.isq_filt = (1.0 - alpha) * state.isq_filt + alpha * isq_med
    else
        state.isd_filt = isd_med
        state.isq_filt = isq_med
    end

    isd_filt = state.isd_filt
    isq_filt = state.isq_filt

    # ------------------------------------------------------------
    # Current reference limiting, d-axis priority
    # ------------------------------------------------------------

    if abs(isd_ref) > p.Is_max
        isd_ref_lim = sign(isd_ref) * p.Is_max
        isq_ref_lim = 0.0
    else
        isd_ref_lim = isd_ref
        isq_max_disp = sqrt(max(p.Is_max^2 - isd_ref_lim^2, 0.0))

        if abs(isq_ref) > isq_max_disp
            isq_ref_lim = sign(isq_ref) * isq_max_disp
        else
            isq_ref_lim = isq_ref
        end
    end

    # ------------------------------------------------------------
    # Errors
    # ------------------------------------------------------------

    err_d = isd_ref_lim - isd_filt
    err_q = isq_ref_lim - isq_filt

    # ------------------------------------------------------------
    # PI regulator
    # ------------------------------------------------------------

    up_d = Kp * err_d
    up_q = Kp * err_q

    vsd_PI = up_d + state.ui_d
    vsq_PI = up_q + state.ui_q

    # ------------------------------------------------------------
    # Feedforward / decoupling
    # ------------------------------------------------------------

    if p.use_feedforward
        vsd_ff = -omega_e * p.sigma_Lss * isq_filt
        vsq_ff =  omega_e * p.sigma_Lss * isd_filt + omega_e * p.k * lambda_rd
    else
        vsd_ff = 0.0
        vsq_ff = 0.0
    end

    # ------------------------------------------------------------
    # Voltage sum
    # ------------------------------------------------------------

    vsd_unsat = vsd_PI + vsd_ff
    vsq_unsat = vsq_PI + vsq_ff

    vs_mod_unsat = sqrt(vsd_unsat^2 + vsq_unsat^2)

    # ------------------------------------------------------------
    # Voltage saturation
    # ------------------------------------------------------------

    if p.use_saturation && vs_mod_unsat > p.Vs_max
        factor = p.Vs_max / (vs_mod_unsat + eps())
        vsd = vsd_unsat * factor
        vsq = vsq_unsat * factor
        saturado = true
    else
        vsd = vsd_unsat
        vsq = vsq_unsat
        saturado = false
    end

    # ------------------------------------------------------------
    # Anti-windup / integrator update
    # ------------------------------------------------------------

    if p.use_antiwindup && saturado
        vsd_PI_sat = vsd - vsd_ff
        vsq_PI_sat = vsq - vsq_ff

        delta_vsd = vsd_PI - vsd_PI_sat
        delta_vsq = vsq_PI - vsq_PI_sat

        state.ui_d += Ki * err_d * Ts - Kaw * delta_vsd
        state.ui_q += Ki * err_q * Ts - Kaw * delta_vsq
    else
        state.ui_d += Ki * err_d * Ts
        state.ui_q += Ki * err_q * Ts
    end

    is_mod = sqrt(isd_filt^2 + isq_filt^2)

    return CurrentControllerDiscreteOutput(
        vsd = vsd,
        vsq = vsq,

        isd_ref_lim = isd_ref_lim,
        isq_ref_lim = isq_ref_lim,

        isd_filt = isd_filt,
        isq_filt = isq_filt,

        err_d = err_d,
        err_q = err_q,

        vsd_PI = vsd_PI,
        vsq_PI = vsq_PI,

        vsd_ff = vsd_ff,
        vsq_ff = vsq_ff,

        vsd_unsat = vsd_unsat,
        vsq_unsat = vsq_unsat,

        vs_mod_unsat = vs_mod_unsat,
        saturado = saturado,

        is_mod = is_mod,
    )
end