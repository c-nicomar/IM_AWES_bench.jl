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

Base.@kwdef mutable struct RotorFluxObserverDiscreteState
    lambda_rd_r::Float64 = 0.0
    lambda_rq_r::Float64 = 0.0

    Te_filt_speed_prev::Float64 = 0.0
    Te_filt_torque_prev::Float64 = 0.0
end

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