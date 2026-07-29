# ============================================================
# Discrete FOC outer loop: torque mode + T1 + F1
# ============================================================
#
# This is a first simplified version of:
#
#   mode_control = 1  -> TORQUE mode
#   mode_torque  = 1  -> T1, torque-to-iq using Kt_nom
#   mode_flux    = 1  -> F1, constant id reference
#
# It generates:
#   isd_ref, isq_ref
#
# to be consumed by current_controller_discrete.jl.
# ============================================================

"""
    OuterTorqueFluxF1State(; kwargs...)

Mutable state of the F1 constant-flux *torque* outer loop, advanced in place by
[`outer_torque_flux_f1_step!`](@ref). This loop has no integrator: its only
memory is the two slew-rate limiters (`ant` for *anterior*, the previous value).

# Fields

- `Te_ref_ant`: previous ramped torque reference in N·m, the starting point for
  this sample's `Te_dot_max` step.
- `id_ref_ant`: previous ramped d-axis current reference in A. The default equals
  the default `isd_nom`, so a fresh state starts at nominal flux instead of
  ramping up from zero.

See also [`OuterTorqueFluxF1Params`](@ref), [`OuterTorqueFluxF1Output`](@ref).
"""
Base.@kwdef mutable struct OuterTorqueFluxF1State
    Te_ref_ant::Float64 = 0.0
    id_ref_ant::Float64 = 23.04579328
end

"""
    OuterTorqueFluxF1Params(; kwargs...)

Machine constants and limits of the F1 constant-flux torque outer loop, consumed
by [`outer_torque_flux_f1_step!`](@ref).

# Fields

- `Ts`: sample time in s. Both slew-rate limits are converted to a per-sample
  step with it.
- `p`: pole pairs.
- `Lm`, `Lrr`: magnetizing and rotor inductance in H. They fix the coupling
  coefficient `k = Lm/Lrr` and hence the nominal torque constant
  `Kt_nom = 1.5*p*k*Lm*isd_nom`.
- `isd_nom`: the constant d-axis current reference in A that defines F1 mode.
  Flux is held at `Lm*isd_nom` at all speeds — there is no field weakening in
  this block.
- `Is_max`: stator current magnitude limit in A, bounding the dq reference
  vector with d-axis priority.
- `isd_min`: lower bound in A on the d-axis reference, so the machine is never
  left without flux.
- `Te_max`: torque reference magnitude limit in N·m.
- `Te_dot_max`: torque reference slew rate in N·m/s.
- `id_dot_max`: d-axis current reference slew rate in A/s. Rotor flux cannot
  change instantly, so the flux command is ramped rather than stepped.

See also [`OuterTorqueFluxF1State`](@ref), [`OuterTorqueFluxF1Output`](@ref).
"""
Base.@kwdef struct OuterTorqueFluxF1Params
    Ts::Float64 = 100e-6

    # Machine / nominal FOC parameters
    p::Float64 = 2.0
    Lm::Float64 = 40.84e-3
    Lrr::Float64 = 45.12e-3

    # F1 flux setting
    isd_nom::Float64 = 23.04579328

    # Limits
    Is_max::Float64 = 40.0
    isd_min::Float64 = 5.0
    Te_max::Float64 = 124.0419647
    Te_dot_max::Float64 = 500.0
    id_dot_max::Float64 = 600.0
end

"""
    OuterTorqueFluxF1Output(; kwargs...)

Result of one [`outer_torque_flux_f1_step!`](@ref) call, returned fresh each
sample. `isd_ref`/`isq_ref` are what the inner current controller consumes; the
rest is diagnostic.

# Fields

- `isd_ref`, `isq_ref`: dq current references in A for
  [`current_controller_step!`](@ref).
- `Te_ref_out`: torque reference in N·m after slew-rate limiting and clamping to
  `Te_max`.
- `lambda_ref_out`: rotor flux reference in Wb. Constant at `Lm*isd_nom` in F1
  mode.
- `e_wm`, `e_Te`, `e_lambda`: always zero here. They exist so torque and speed
  outer loops share an output layout; only the speed loops fill in `e_wm`.
- `sat_isd`, `sat_isq`, `sat_Te`: saturation flags, `+1` at the upper limit,
  `-1` at the lower limit, `0` when unlimited.
- `T_load_est`: the `TL` argument passed through unchanged, logged for
  comparison. This block does not use it.
- `Kt_nom`: nominal torque constant `1.5*p*k*Lm*isd_nom` in N·m/A used for the
  torque-to-`isq` map.
- `isq_max_disp`: q-axis current in A still available inside the `Is_max` circle
  after the d-axis has taken its share (*disponible*, available).

See also [`OuterTorqueFluxF1Params`](@ref).
"""
Base.@kwdef struct OuterTorqueFluxF1Output
    isd_ref::Float64 = 0.0
    isq_ref::Float64 = 0.0

    Te_ref_out::Float64 = 0.0
    lambda_ref_out::Float64 = 0.0

    e_wm::Float64 = 0.0
    e_Te::Float64 = 0.0
    e_lambda::Float64 = 0.0

    sat_isd::Int = 0
    sat_isq::Int = 0
    sat_Te::Int = 0

    T_load_est::Float64 = 0.0

    Kt_nom::Float64 = 0.0
    isq_max_disp::Float64 = 0.0
end

function reset!(state::OuterTorqueFluxF1State, p::OuterTorqueFluxF1Params)
    state.Te_ref_ant = 0.0
    state.id_ref_ant = p.isd_nom
    return nothing
end

"""
    outer_torque_flux_f1_step!(state, p; Te_ref_ext, TL = 0.0, reset = false)

Advance the F1 constant-flux torque outer loop by one sample period `p.Ts` and
return the dq current references the inner current controller should track.

This is the `mode_control = 1` (TORQUE) + `mode_torque = 1` (T1) +
`mode_flux = 1` (F1) path of the MATLAB original: an externally commanded torque
is slew- and magnitude-limited, then converted to `isq` through the *nominal*
torque constant, while `isd` is held at nominal flux. There is no feedback here
— the block is a feedforward reference shaper, so it has no integrator and
cannot wind up.

`state` is mutated in place; the result is a fresh
[`OuterTorqueFluxF1Output`](@ref).

# Arguments

- `state::OuterTorqueFluxF1State`: the two slew-rate limiter memories, mutated.
- `p::OuterTorqueFluxF1Params`: machine constants and limits.
- `Te_ref_ext`: externally commanded electromagnetic torque in N·m. Positive
  accelerates toward positive speed, per `J*dω/dt = Te + TL - B*ω`.
- `TL`: load torque in N·m, passed through to `T_load_est` for logging only.
  Same sign convention: positive `TL` also accelerates toward positive speed.
- `reset`: when `true`, reinitialise the ramp states and seed the torque ramp at
  `Te_ref_ext`, so the first step does not ramp up from zero.

# Steps

1. **Torque reference shaping.** `Te_ref_ext` is ramped at `Te_dot_max` and then
   clamped to `±Te_max`, setting `sat_Te`.
2. **F1 flux reference.** `isd` targets the constant `isd_nom`, ramped at
   `id_dot_max` and clamped to `[isd_min, Is_max]`.
3. **Torque-to-current map.** `isq_ref = Te_ref_out / Kt_nom` with the *nominal*
   torque constant. Because flux is constant in F1, this map is exact as long as
   the actual flux equals `Lm*isd_nom`.
4. **Current-circle limiting, d-axis priority.** `isq_ref` is clamped to
   `sqrt(Is_max^2 - isd_ref^2)`, so flux is preserved and torque gives way.

See also [`outer_speed_flux_f1_step!`](@ref) for the speed-controlled
counterpart, and [`outer_speed_flux_mtpa_step!`](@ref) for the variable-flux
alternative.
"""
function outer_torque_flux_f1_step!(
    state::OuterTorqueFluxF1State,
    p::OuterTorqueFluxF1Params;
    Te_ref_ext::Float64,
    TL::Float64 = 0.0,
    reset::Bool = false,
)

    if reset
        reset!(state, p)
        state.Te_ref_ant = Te_ref_ext
    end

    # ------------------------------------------------------------
    # Nominal constants
    # ------------------------------------------------------------

    k = p.Lm / p.Lrr
    lambda_rd_nom = p.Lm * p.isd_nom
    Kt_nom = 1.5 * p.p * k * lambda_rd_nom

    # ------------------------------------------------------------
    # Torque reference ramp and saturation
    #
    # This is the mode_control = 1 path in the MATLAB function:
    # external Te_ref with Te_dot_max and Te_max limits.
    # ------------------------------------------------------------

    delta_Te_max = p.Te_dot_max * p.Ts
    delta_Te = Te_ref_ext - state.Te_ref_ant
    delta_Te = clamp(delta_Te, -delta_Te_max, delta_Te_max)

    Te_ref_ramp = state.Te_ref_ant + delta_Te
    state.Te_ref_ant = Te_ref_ramp

    if Te_ref_ramp > p.Te_max
        Te_ref_out = p.Te_max
        sat_Te = 1
    elseif Te_ref_ramp < -p.Te_max
        Te_ref_out = -p.Te_max
        sat_Te = -1
    else
        Te_ref_out = Te_ref_ramp
        sat_Te = 0
    end

    # ------------------------------------------------------------
    # F1 flux mode: constant d-axis current reference
    #
    # The MATLAB function then applies an id ramp and saturation.
    # ------------------------------------------------------------

    isd_desired = p.isd_nom
    lambda_ref_out = lambda_rd_nom

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

    # ------------------------------------------------------------
    # T1 torque mode: simple nominal torque constant
    #
    # isq_ref_unsat = Te_ref / Kt_nom
    # ------------------------------------------------------------

    isq_ref_unsat = Te_ref_out / Kt_nom

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

    return OuterTorqueFluxF1Output(
        isd_ref = isd_ref,
        isq_ref = isq_ref,

        Te_ref_out = Te_ref_out,
        lambda_ref_out = lambda_ref_out,

        e_wm = 0.0,
        e_Te = 0.0,
        e_lambda = 0.0,

        sat_isd = sat_isd,
        sat_isq = sat_isq,
        sat_Te = sat_Te,

        T_load_est = TL,

        Kt_nom = Kt_nom,
        isq_max_disp = isq_max_disp,
    )
end