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

Base.@kwdef mutable struct OuterTorqueFluxF1State
    Te_ref_ant::Float64 = 0.0
    id_ref_ant::Float64 = 23.04579328
end

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