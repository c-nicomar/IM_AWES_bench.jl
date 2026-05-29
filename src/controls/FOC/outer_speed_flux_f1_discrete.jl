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

Base.@kwdef mutable struct OuterSpeedFluxF1State
    ui_wm::Float64 = 0.0
    wm_filt::Float64 = 0.0
    wm_ref_ant::Float64 = 0.0
    id_ref_ant::Float64 = 23.04579328
end

Base.@kwdef struct OuterSpeedFluxF1Params
    Ts::Float64 = 100e-6

    # Machine / nominal FOC parameters
    p::Float64 = 2.0
    Lm::Float64 = 40.84e-3
    Lrr::Float64 = 45.12e-3
    J::Float64 = 0.2685 + 0.1
    B::Float64 = 0.01298

    # F1 flux setting
    isd_nom::Float64 = 23.04579328

    # Limits
    Is_max::Float64 = 40.0
    isd_min::Float64 = 5.0
    Te_max::Float64 = 124.0419647
    wm_dot_max::Float64 = 100.0
    id_dot_max::Float64 = 600.0

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

function outer_speed_flux_f1_step!(
    state::OuterSpeedFluxF1State,
    p::OuterSpeedFluxF1Params;
    wm_ref::Float64,
    wm_med::Float64,
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
    # F1 flux mode: constant d-axis current reference
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

        Te_PI = Te_PI,
        Te_ff = Te_ff,
    )
end