# ============================================================
# Discrete rotor-flux and torque observer
# ============================================================
#
# Based on the MATLAB function obs_flujo_L1_alfabeta.
#
# This observer:
#   1. takes alpha-beta stator currents and mechanical angle,
#   2. transforms currents to rotor frame,
#   3. filters rotor flux with two first-order discrete filters,
#   4. computes estimated flux angle,
#   5. projects currents into flux frame,
#   6. computes estimated electromagnetic torque.
# ============================================================

"""
    RotorFluxObserverDiscreteState(; kwargs...)

Mutable state of the discrete rotor-flux and torque observer, advanced in place
by [`rotor_flux_observer_step!`](@ref).

The flux estimate is held in the *rotor* reference frame, where the rotor flux
dynamics reduce to a plain first-order lag driven by the rotor-frame currents.
That is what makes the observer two scalar filters rather than a matrix
integration.

# Fields

- `lambda_rd_r`, `lambda_rq_r`: rotor flux linkage in Wb, expressed in the rotor
  frame (trailing `_r`). Not to be confused with the `_e` quantities in the
  output, which are in the estimated flux frame.
- `Te_filt_speed_prev`, `Te_filt_torque_prev`: previous outputs in N·m of the
  two independent optional torque filters. They exist so a speed loop and a
  torque loop can consume differently smoothed versions of the same estimate.

See also [`RotorFluxObserverDiscreteParams`](@ref),
[`RotorFluxObserverDiscreteOutput`](@ref).
"""
Base.@kwdef mutable struct RotorFluxObserverDiscreteState
    lambda_rd_r::Float64 = 0.0
    lambda_rq_r::Float64 = 0.0

    Te_filt_speed_prev::Float64 = 0.0
    Te_filt_torque_prev::Float64 = 0.0
end

"""
    RotorFluxObserverDiscreteParams(; kwargs...)

Machine constants of the discrete rotor-flux and torque observer, consumed by
[`rotor_flux_observer_step!`](@ref).

These are the observer's *assumed* machine parameters. The hybrid simulators
deliberately allow them to differ from the plant's, through their `obs_*_scale`
keywords, which is how rotor-resistance detuning is studied: `tau_r` depends on
`Rr`, and a wrong `tau_r` biases the estimated flux angle and hence the whole
field-oriented decomposition.

# Fields

- `Lm`: magnetizing inductance in H, the gain from rotor-frame current to rotor
  flux.
- `Lss`: stator self-inductance in H, used only for the stator flux output.
- `Lrr`: rotor self-inductance in H. Sets the coupling coefficient `k = Lm/Lrr`
  in the torque estimate.
- `tau_r`: rotor time constant `Lrr/Rr` in s, the pole of both flux filters and
  the gain of the slip estimate. Internally floored at `Ts`.
- `p`: pole pairs, converting mechanical angle and speed to electrical.
- `Ts`: sample time in s.
- `tau_Te_speed`: time constant in s of the torque filter intended for a speed
  loop. `0` (the default) passes `Te_raw` through unfiltered.
- `tau_Te_torque`: same for the torque-loop consumer. `0` disables it.
- `flux_eps`: small flux magnitude in Wb below which the angle is not computed
  (`theta_e` is held at `0`), and which is added to the denominator of the slip
  estimate. It keeps the observer finite while the machine is still unfluxed at
  startup.

See also [`RotorFluxObserverDiscreteState`](@ref),
[`RotorFluxObserverDiscreteOutput`](@ref).
"""
Base.@kwdef struct RotorFluxObserverDiscreteParams
    Lm::Float64 = 0.04084
    Lss::Float64 = 0.04512
    Lrr::Float64 = 0.04512
    tau_r::Float64 = 0.3869971696
    p::Float64 = 2.0
    Ts::Float64 = 100e-6

    tau_Te_speed::Float64 = 0.0
    tau_Te_torque::Float64 = 0.0

    flux_eps::Float64 = 1e-6
end

"""
    RotorFluxObserverDiscreteOutput(; kwargs...)

Result of one [`rotor_flux_observer_step!`](@ref) call, returned fresh each
sample.

This is the block that makes the rest of the loop field-oriented: `theta_e` is
the angle everything else rotates by, and `i_sd_e`/`i_sq_e` are the dq currents
the controllers regulate. A trailing `_e` means "in the estimated flux frame",
`_r` means "in the rotor frame", and `alpha`/`beta` means the stationary frame.

# Fields

- `lambda_rd_e`, `lambda_rq_e`: rotor flux linkage in Wb in the estimated flux
  frame. By construction `lambda_rq_e` is ~0 and `lambda_rd_e` carries the whole
  flux — how close it stays to zero is a direct check on the observer.
- `theta_e`: estimated rotor flux angle in rad, wrapped to `[0, 2π)`. Used for
  the Park and inverse-Park transforms in the loop.
- `Te_raw`: estimated electromagnetic torque in N·m, `1.5*p*k*lambda_rd_e*i_sq_e`.
  Positive accelerates toward positive speed, per `J*dω/dt = Te + TL - B*ω`.
- `Te_filt_speed`, `Te_filt_torque`: the two independently filtered torque
  estimates in N·m. Equal to `Te_raw` when the corresponding `tau_Te_*` is `0`.
- `flux_r_mod`: rotor flux magnitude in Wb in the stationary frame.
- `i_sd_e`, `i_sq_e`: stator currents in A projected onto the estimated flux
  frame — the flux-producing and torque-producing components fed back to
  [`current_controller_step!`](@ref).
- `psi_sd_e`, `psi_sq_e`: stator flux linkage in Wb in the same frame.
- `flux_s_mod`: stator flux magnitude in Wb.
- `i_alpha`, `i_beta`: the stationary-frame input currents in A, passed through
  for logging.
- `i_sd_r`, `i_sq_r`: stator currents in A in the rotor frame, the inputs to the
  two flux filters.
- `lambda_r_alpha`, `lambda_r_beta`: rotor flux in Wb back in the stationary
  frame, from which `theta_e` is taken.
- `omega_sl`: estimated slip speed in electrical rad/s.
- `omega_e`: estimated synchronous electrical speed in rad/s, `p*omega_m +
  omega_sl`. This is what the current controller's decoupling feedforward and
  the F1 field-weakening logic should be fed.

See also [`RotorFluxObserverDiscreteParams`](@ref).
"""
Base.@kwdef struct RotorFluxObserverDiscreteOutput
    lambda_rd_e::Float64 = 0.0
    lambda_rq_e::Float64 = 0.0

    theta_e::Float64 = 0.0

    Te_raw::Float64 = 0.0
    Te_filt_speed::Float64 = 0.0
    Te_filt_torque::Float64 = 0.0

    flux_r_mod::Float64 = 0.0

    i_sd_e::Float64 = 0.0
    i_sq_e::Float64 = 0.0

    psi_sd_e::Float64 = 0.0
    psi_sq_e::Float64 = 0.0
    flux_s_mod::Float64 = 0.0

    i_alpha::Float64 = 0.0
    i_beta::Float64 = 0.0

    i_sd_r::Float64 = 0.0
    i_sq_r::Float64 = 0.0

    lambda_r_alpha::Float64 = 0.0
    lambda_r_beta::Float64 = 0.0

    omega_sl::Float64 = 0.0
    omega_e::Float64 = 0.0
end

function reset!(state::RotorFluxObserverDiscreteState)
    state.lambda_rd_r = 0.0
    state.lambda_rq_r = 0.0
    state.Te_filt_speed_prev = 0.0
    state.Te_filt_torque_prev = 0.0
    return nothing
end

"""
    rotor_flux_observer_step!(state, p; i_alpha, i_beta, theta_m, omega_m,
                              reset = false)

Advance the discrete rotor-flux and torque observer by one sample period `p.Ts`.

This is the first block of every hybrid iteration: it turns measured
stationary-frame currents and the mechanical angle into the flux angle
`theta_e`, the dq currents, the flux estimate and the torque estimate that all
downstream blocks consume. It is the Julia counterpart of the MATLAB function
`obs_flujo_L1_alfabeta`.

It is a *current-model* observer — it uses currents and rotor position only, no
voltage measurement — so it works down to zero speed but inherits the accuracy
of the assumed `tau_r`.

`state` is mutated in place; the result is a fresh
[`RotorFluxObserverDiscreteOutput`](@ref).

# Arguments

- `state::RotorFluxObserverDiscreteState`: rotor-frame flux estimate and the two
  torque-filter memories, mutated.
- `p::RotorFluxObserverDiscreteParams`: assumed machine constants.
- `i_alpha`, `i_beta`: measured stator currents in A in the stationary
  alpha-beta frame.
- `theta_m`: mechanical rotor angle in rad. Multiplied by `p.p` to get the
  electrical rotor angle; it need not be wrapped.
- `omega_m`: mechanical rotor speed in rad/s, used only for `omega_e`.
- `reset`: when `true`, zero the flux estimate and both torque filters before
  stepping.

# Steps

1. **Stationary → rotor frame.** Rotate the currents by `theta_r = p*theta_m`.
2. **Flux filters.** In the rotor frame the rotor flux obeys
   `tau_r*dλ/dt + λ = Lm*i`, discretised as
   `λ ← (1-β)*λ + β*Lm*i` with `β = Ts/tau_r` clamped to `[0,1]`. Two scalars,
   one per axis.
3. **Rotor → stationary frame**, giving `lambda_r_alpha`/`lambda_r_beta` and
   hence `theta_e = atan(lambda_r_beta, lambda_r_alpha)`, wrapped to `[0, 2π)`.
   Below `flux_eps` the angle is held at `0` rather than taken from noise.
4. **Stationary → estimated flux frame.** Project currents and fluxes onto
   `theta_e`, producing `i_sd_e`, `i_sq_e`, `lambda_rd_e` and `lambda_rq_e`.
5. **Torque and stator flux.** `Te_raw = 1.5*p*k*lambda_rd_e*i_sq_e` with
   `k = Lm/Lrr`, then the two optional one-pole torque filters.
6. **Slip and synchronous speed.** `omega_sl = (Lm*i_sq_e)/(tau_r*lambda_rd_e)`,
   with `flux_eps` guarding the denominator, and
   `omega_e = p*omega_m + omega_sl`.

See also [`current_controller_step!`](@ref), which consumes `i_sd_e`, `i_sq_e`,
`omega_e` and `lambda_rd_e`, and [`load_torque_kalman_step!`](@ref), which
consumes the torque estimate.
"""
function rotor_flux_observer_step!(
    state::RotorFluxObserverDiscreteState,
    p::RotorFluxObserverDiscreteParams;
    i_alpha::Float64,
    i_beta::Float64,
    theta_m::Float64,
    omega_m::Float64,
    reset::Bool = false,
)

    if reset
        state.lambda_rd_r = 0.0
        state.lambda_rq_r = 0.0
        state.Te_filt_speed_prev = 0.0
        state.Te_filt_torque_prev = 0.0
    end

    tau_r = max(p.tau_r, p.Ts)

    # ------------------------------------------------------------
    # Stationary alpha-beta -> rotor frame dq_r
    # ------------------------------------------------------------

    theta_r = p.p * theta_m

    cos_r = cos(theta_r)
    sin_r = sin(theta_r)

    i_sd_r =  cos_r * i_alpha + sin_r * i_beta
    i_sq_r = -sin_r * i_alpha + cos_r * i_beta

    # ------------------------------------------------------------
    # Discrete rotor-flux observer in rotor frame
    # ------------------------------------------------------------

    beta_r = p.Ts / tau_r
    beta_r = clamp(beta_r, 0.0, 1.0)
    alpha_r = 1.0 - beta_r

    state.lambda_rd_r = alpha_r * state.lambda_rd_r + beta_r * p.Lm * i_sd_r
    state.lambda_rq_r = alpha_r * state.lambda_rq_r + beta_r * p.Lm * i_sq_r

    # ------------------------------------------------------------
    # Rotor flux back to stationary alpha-beta
    # ------------------------------------------------------------

    lambda_r_alpha = cos_r * state.lambda_rd_r - sin_r * state.lambda_rq_r
    lambda_r_beta  = sin_r * state.lambda_rd_r + cos_r * state.lambda_rq_r

    flux_r_mod = sqrt(lambda_r_alpha^2 + lambda_r_beta^2)

    if flux_r_mod > p.flux_eps
        theta_e = atan(lambda_r_beta, lambda_r_alpha)
    else
        theta_e = 0.0
    end

    if theta_e < 0.0
        theta_e += 2π
    end

    # ------------------------------------------------------------
    # Stationary alpha-beta -> estimated flux frame dq_e
    # ------------------------------------------------------------

    cos_e = cos(theta_e)
    sin_e = sin(theta_e)

    i_sd_e =  cos_e * i_alpha + sin_e * i_beta
    i_sq_e = -sin_e * i_alpha + cos_e * i_beta

    lambda_rd_e =  cos_e * lambda_r_alpha + sin_e * lambda_r_beta
    lambda_rq_e = -sin_e * lambda_r_alpha + cos_e * lambda_r_beta

    # ------------------------------------------------------------
    # Estimated electromagnetic torque and stator flux
    # ------------------------------------------------------------

    k = p.Lm / p.Lrr

    Te_raw = 1.5 * p.p * k * lambda_rd_e * i_sq_e

    psi_sd_e = p.Lss * i_sd_e + k * lambda_rd_e
    psi_sq_e = p.Lss * i_sq_e + k * lambda_rq_e

    flux_s_mod = sqrt(psi_sd_e^2 + psi_sq_e^2)

    # ------------------------------------------------------------
    # Optional torque filtering
    # ------------------------------------------------------------

    if p.tau_Te_speed > 0.0
        a = p.Ts / (p.tau_Te_speed + p.Ts)
        Te_filt_speed = (1.0 - a) * state.Te_filt_speed_prev + a * Te_raw
        state.Te_filt_speed_prev = Te_filt_speed
    else
        Te_filt_speed = Te_raw
    end

    if p.tau_Te_torque > 0.0
        a2 = p.Ts / (p.tau_Te_torque + p.Ts)
        Te_filt_torque = (1.0 - a2) * state.Te_filt_torque_prev + a2 * Te_raw
        state.Te_filt_torque_prev = Te_filt_torque
    else
        Te_filt_torque = Te_raw
    end

    # ------------------------------------------------------------
    # Slip and synchronous electrical speed estimate
    # ------------------------------------------------------------

    omega_sl = (1.0 / tau_r) * (p.Lm * i_sq_e) / (lambda_rd_e + p.flux_eps)
    omega_e = p.p * omega_m + omega_sl

    return RotorFluxObserverDiscreteOutput(
        lambda_rd_e = lambda_rd_e,
        lambda_rq_e = lambda_rq_e,

        theta_e = theta_e,

        Te_raw = Te_raw,
        Te_filt_speed = Te_filt_speed,
        Te_filt_torque = Te_filt_torque,

        flux_r_mod = flux_r_mod,

        i_sd_e = i_sd_e,
        i_sq_e = i_sq_e,

        psi_sd_e = psi_sd_e,
        psi_sq_e = psi_sq_e,
        flux_s_mod = flux_s_mod,

        i_alpha = i_alpha,
        i_beta = i_beta,

        i_sd_r = i_sd_r,
        i_sq_r = i_sq_r,

        lambda_r_alpha = lambda_r_alpha,
        lambda_r_beta = lambda_r_beta,

        omega_sl = omega_sl,
        omega_e = omega_e,
    )
end