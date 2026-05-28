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

Base.@kwdef mutable struct CurrentControllerDiscreteState
    ui_d::Float64 = 0.0
    ui_q::Float64 = 0.0
    isd_filt::Float64 = 0.0
    isq_filt::Float64 = 0.0
end

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