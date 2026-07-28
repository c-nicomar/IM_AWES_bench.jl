# ============================================================
# Hybrid FOC speed-control simulator for the 160 kW im IM
# ============================================================
#
# Controller structure:
#
#   speed reference
#       -> outer_speed_flux_f1_step!
#       -> current_controller_step!
#       -> inverse Park transformation
#       -> stationary alpha-beta induction-machine plant
#
# Estimation:
#
#   rotor_flux_observer_step!
#   load_torque_kalman_step!  (optional)
#
# The physical plant, observer, outer controller, and current controller
# receive independent parameter sets through the nominal values and mismatch
# multipliers. This preserves the package's robustness-testing capability.
#
# Mechanical sign convention:
#
#   J*dωm/dt = Te + Tload - B*ωm
#
# Therefore:
#
#   * positive Tload accelerates the shaft in the positive direction;
#   * negative Tload opposes positive rotation;
#   * positive Tload opposes negative rotation.
#
# The default load profile below applies opposing load steps during both
# the positive- and negative-speed plateaus.
#
# Important package detail:
#
# The hybrid simulators use IMPlantParams, IMPlantState, im_torque, and
# rk4_step, defined by hybrid_foc_current_simulator.jl. These implement the
# same stationary alpha-beta induction-machine equations as
# induction_machine_alpha_beta.jl, but in fixed-step form suitable for
# sample-by-sample discrete control.
# ============================================================


"""
    im_160kw_speed_reference(t; kwargs...)

Bipolar mechanical-speed reference in rad/s:

1. zero-speed flux build-up;
2. ramp from 0 to +`wm_abs`;
3. positive-speed hold;
4. ramp back to zero;
5. zero-speed dwell;
6. ramp from 0 to -`wm_abs`;
7. negative-speed hold;
8. ramp back to zero.

The default profile reaches +50 rad/s and -50 rad/s.
"""
function im_160kw_speed_reference(
    t;
    wm_abs::Float64 = 50.0,

    t_positive_ramp_start::Float64 = 1.0,
    t_positive_ramp_end::Float64 = 2.0,
    t_positive_hold_end::Float64 = 5.0,
    t_positive_return_end::Float64 = 6.0,

    t_negative_ramp_start::Float64 = 7.0,
    t_negative_ramp_end::Float64 = 8.0,
    t_negative_hold_end::Float64 = 11.0,
    t_negative_return_end::Float64 = 12.0,
)
    wm_abs >= 0.0 ||
        throw(ArgumentError("wm_abs must be non-negative."))

    t_positive_ramp_start <= t_positive_ramp_end <=
        t_positive_hold_end <= t_positive_return_end <=
        t_negative_ramp_start <= t_negative_ramp_end <=
        t_negative_hold_end <= t_negative_return_end ||
        throw(ArgumentError("Speed-profile times must be ordered."))

    if t < t_positive_ramp_start
        return 0.0

    elseif t < t_positive_ramp_end
        α = (t - t_positive_ramp_start) /
            (t_positive_ramp_end - t_positive_ramp_start)
        return α * wm_abs

    elseif t < t_positive_hold_end
        return wm_abs

    elseif t < t_positive_return_end
        α = (t - t_positive_hold_end) /
            (t_positive_return_end - t_positive_hold_end)
        return (1.0 - α) * wm_abs

    elseif t < t_negative_ramp_start
        return 0.0

    elseif t < t_negative_ramp_end
        α = (t - t_negative_ramp_start) /
            (t_negative_ramp_end - t_negative_ramp_start)
        return -α * wm_abs

    elseif t < t_negative_hold_end
        return -wm_abs

    elseif t < t_negative_return_end
        α = (t - t_negative_hold_end) /
            (t_negative_return_end - t_negative_hold_end)
        return -(1.0 - α) * wm_abs

    else
        return 0.0
    end
end


"""
    im_160kw_load_torque_profile(t; kwargs...)

Opposing load-torque steps for the bipolar speed test.

At positive speed:
- `-Tload_step1`
- `-Tload_step2`

At negative speed:
- `+Tload_step1`
- `+Tload_step2`

This sign choice follows:

    J*dωm/dt = Te + Tload - B*ωm

and therefore makes the load oppose the commanded direction in both halves of
the test.
"""
function im_160kw_load_torque_profile(
    t;
    Tload_step1::Float64 = 200.0,
    Tload_step2::Float64 = 400.0,

    t_positive_step1::Float64 = 2.5,
    t_positive_step2::Float64 = 3.5,
    t_positive_release::Float64 = 4.5,

    t_negative_step1::Float64 = 8.5,
    t_negative_step2::Float64 = 9.5,
    t_negative_release::Float64 = 10.5,
)
    Tload_step1 >= 0.0 ||
        throw(ArgumentError("Tload_step1 must be non-negative."))

    Tload_step2 >= Tload_step1 ||
        throw(ArgumentError(
            "Tload_step2 must be greater than or equal to Tload_step1.",
        ))

    t_positive_step1 <= t_positive_step2 <= t_positive_release <=
        t_negative_step1 <= t_negative_step2 <= t_negative_release ||
        throw(ArgumentError("Load-profile times must be ordered."))

    if t < t_positive_step1
        return 0.0

    elseif t < t_positive_step2
        return -Tload_step1

    elseif t < t_positive_release
        return -Tload_step2

    elseif t < t_negative_step1
        return 0.0

    elseif t < t_negative_step2
        return Tload_step1

    elseif t < t_negative_release
        return Tload_step2

    else
        return 0.0
    end
end


"""
    simulate_foc_speed_f1_im_160kw(; kwargs...)

Run the 160 kW im induction machine with:

- stationary alpha-beta induction-machine plant;
- discrete rotor-flux and torque observer;
- discrete F1 outer speed/flux controller;
- discrete dq current controller;
- optional load-torque estimator.

The default machine and control values come from the supplied
`pre_kitepower_IM.m` and `IM__FOC_kitepower.slx` model.

## Parameter independence

Nominal values define the controller-design point. The actual plant, observer,
outer controller, current controller, and load estimator can then be perturbed
independently through their corresponding scale factors.

For example, a rotor-resistance robustness test can use:

    plant_Rr_scale = 1.30
    obs_tau_r_scale = 1.00

so the simulated rotor resistance is 30% above the observer assumption.
"""
function simulate_foc_speed_f1_im_160kw(;
    # ------------------------------------------------------------
    # Simulation
    # ------------------------------------------------------------
    t_end::Float64 = 13.0,
    Ts::Float64 = 100e-6,
    plant_substeps::Int = 1,

    # ------------------------------------------------------------
    # Bipolar speed-reference profile
    # ------------------------------------------------------------
    wm_ref_abs::Float64 = 50.0,

    t_positive_ramp_start::Float64 = 1.0,
    t_positive_ramp_end::Float64 = 2.0,
    t_positive_hold_end::Float64 = 5.0,
    t_positive_return_end::Float64 = 6.0,

    t_negative_ramp_start::Float64 = 7.0,
    t_negative_ramp_end::Float64 = 8.0,
    t_negative_hold_end::Float64 = 11.0,
    t_negative_return_end::Float64 = 12.0,

    # ------------------------------------------------------------
    # Opposing load-torque steps
    # ------------------------------------------------------------
    Tload_step1::Float64 = 200.0,
    Tload_step2::Float64 = 400.0,

    t_positive_load_step1::Float64 = 2.5,
    t_positive_load_step2::Float64 = 3.5,
    t_positive_load_release::Float64 = 4.5,

    t_negative_load_step1::Float64 = 8.5,
    t_negative_load_step2::Float64 = 9.5,
    t_negative_load_release::Float64 = 10.5,

    # ------------------------------------------------------------
    # Load estimator and load feedforward
    # ------------------------------------------------------------
    # :none   -> TL_est = 0
    # :actual -> use the exact simulated Tload
    # :kalman -> use load_torque_kalman_step!
    load_estimator::Symbol = :actual,

    use_load_feedforward::Bool = true,
    load_ff_sign::Float64 = -1.0,

    TL_kalman_R::Float64 = 0.01,
    TL_kalman_q_omega::Float64 = 0.1,
    TL_kalman_q_TL::Float64 = 200.0,
    TL_kalman_limit_positive::Bool = false,

    # ------------------------------------------------------------
    # Nominal 160 kW machine parameters
    #
    # Datasheet equivalent circuit, 400 V delta, converted using /3
    # as in the supplied Simulink initialization script.
    # ------------------------------------------------------------
    fn::Float64 = 50.0,
    Rs::Float64 = 0.0203 / 3.0,
    Rr::Float64 = 0.0207 / 3.0,
    Xls_50::Float64 = 0.1421 / 3.0,
    Xlr_50::Float64 = 0.2588 / 3.0,
    Xm_50::Float64 = 4.6 / 3.0,
    p_pairs::Float64 = 3.0,

    # Mechanical parameters from the supplied initialization script.
    J::Float64 = 130.0 / 13.1^2,
    B::Float64 = 0.0,

    # ------------------------------------------------------------
    # F1 outer-loop parameters
    # ------------------------------------------------------------
    isd_nom::Float64 = 160.0 * sqrt(2.0),
    outer_Is_max::Float64 = 310.0 * sqrt(2.0),
    isd_min::Float64 = 0.25 * 160.0 * sqrt(2.0),
    Te_max::Float64 = 1540.0,

    wm_dot_max::Float64 = 100.0,
    id_dot_max::Float64 = 600.0,

    use_field_weakening::Bool = true,
    wm_base_fw::Float64 = 992.0 * 2π / 60.0,

    outer_tau_f_wm::Float64 = 10e-3,
    outer_ts_wm::Float64 = 500e-3,
    outer_ts_dist_wm::Float64 = 3.0,

    # ------------------------------------------------------------
    # Inner current-controller parameters
    # ------------------------------------------------------------
    Vdc::Float64 = 605.0,
    Vs_max::Float64 = Vdc / sqrt(3.0),

    # The Simulink current-controller input uses Is_lim*0.95.
    Is_max::Float64 = 0.95 * 310.0 * sqrt(2.0),

    current_ts_spec::Float64 = 2e-3,
    current_tau_f::Float64 = 50e-6,

    use_filter::Bool = true,
    use_feedforward::Bool = true,
    use_saturation::Bool = true,
    use_antiwindup::Bool = true,

    # ------------------------------------------------------------
    # Actual-plant mismatch multipliers
    # ------------------------------------------------------------
    plant_Rs_scale::Float64 = 1.0,
    plant_Rr_scale::Float64 = 1.0,
    plant_Lls_scale::Float64 = 1.0,
    plant_Llr_scale::Float64 = 1.0,
    plant_Lm_scale::Float64 = 1.0,
    plant_J_scale::Float64 = 1.0,
    plant_B_scale::Float64 = 1.0,

    # ------------------------------------------------------------
    # Rotor-flux observer assumed-parameter multipliers
    # ------------------------------------------------------------
    obs_Lm_scale::Float64 = 1.0,
    obs_Lss_scale::Float64 = 1.0,
    obs_Lrr_scale::Float64 = 1.0,
    obs_tau_r_scale::Float64 = 1.0,
    obs_tau_Te_speed::Float64 = 0.0,
    obs_tau_Te_torque::Float64 = 0.0,

    # ------------------------------------------------------------
    # Outer-controller assumed-parameter multipliers
    # ------------------------------------------------------------
    outer_Lm_scale::Float64 = 1.0,
    outer_Lrr_scale::Float64 = 1.0,
    outer_J_scale::Float64 = 1.0,
    outer_B_scale::Float64 = 1.0,

    # ------------------------------------------------------------
    # Current-controller assumed-parameter multipliers
    # ------------------------------------------------------------
    ctrl_Rs_scale::Float64 = 1.0,
    ctrl_sigma_Lss_scale::Float64 = 1.0,
    ctrl_k_scale::Float64 = 1.0,

    # ------------------------------------------------------------
    # Load-estimator assumed-mechanical-parameter multipliers
    # ------------------------------------------------------------
    load_est_J_scale::Float64 = 1.0,
    load_est_B_scale::Float64 = 1.0,
)
    # ============================================================
    # Input validation
    # ============================================================
    t_end > 0.0 ||
        throw(ArgumentError("t_end must be positive."))

    Ts > 0.0 ||
        throw(ArgumentError("Ts must be positive."))

    plant_substeps >= 1 ||
        throw(ArgumentError("plant_substeps must be at least 1."))

    fn > 0.0 ||
        throw(ArgumentError("fn must be positive."))

    Rs > 0.0 ||
        throw(ArgumentError("Rs must be positive."))

    Rr > 0.0 ||
        throw(ArgumentError("Rr must be positive."))

    Xls_50 >= 0.0 ||
        throw(ArgumentError("Xls_50 must be non-negative."))

    Xlr_50 >= 0.0 ||
        throw(ArgumentError("Xlr_50 must be non-negative."))

    Xm_50 > 0.0 ||
        throw(ArgumentError("Xm_50 must be positive."))

    p_pairs >= 1.0 ||
        throw(ArgumentError("p_pairs must be at least 1."))

    J > 0.0 ||
        throw(ArgumentError("J must be positive."))

    B >= 0.0 ||
        throw(ArgumentError("B must be non-negative."))

    Vs_max > 0.0 ||
        throw(ArgumentError("Vs_max must be positive."))

    Is_max > 0.0 ||
        throw(ArgumentError("Is_max must be positive."))

    outer_Is_max > 0.0 ||
        throw(ArgumentError("outer_Is_max must be positive."))

    Te_max > 0.0 ||
        throw(ArgumentError("Te_max must be positive."))

    load_estimator in (:none, :actual, :kalman) ||
        throw(ArgumentError(
            "load_estimator must be :none, :actual, or :kalman.",
        ))

    # Validate the profiles before entering the simulation loop.
    im_160kw_speed_reference(
        0.0;
        wm_abs = wm_ref_abs,
        t_positive_ramp_start = t_positive_ramp_start,
        t_positive_ramp_end = t_positive_ramp_end,
        t_positive_hold_end = t_positive_hold_end,
        t_positive_return_end = t_positive_return_end,
        t_negative_ramp_start = t_negative_ramp_start,
        t_negative_ramp_end = t_negative_ramp_end,
        t_negative_hold_end = t_negative_hold_end,
        t_negative_return_end = t_negative_return_end,
    )

    im_160kw_load_torque_profile(
        0.0;
        Tload_step1 = Tload_step1,
        Tload_step2 = Tload_step2,
        t_positive_step1 = t_positive_load_step1,
        t_positive_step2 = t_positive_load_step2,
        t_positive_release = t_positive_load_release,
        t_negative_step1 = t_negative_load_step1,
        t_negative_step2 = t_negative_load_step2,
        t_negative_release = t_negative_load_release,
    )

    # ============================================================
    # Nominal electrical parameters
    # ============================================================
    ω_nom = 2π * fn

    Lls_nom = Xls_50 / ω_nom
    Llr_nom = Xlr_50 / ω_nom
    Lm_nom = Xm_50 / ω_nom

    Lss_nom = Lls_nom + Lm_nom
    Lrr_nom = Llr_nom + Lm_nom

    sigma_Lss_nom = Lss_nom - Lm_nom^2 / Lrr_nom
    k_nom = Lm_nom / Lrr_nom
    tau_r_nom = Lrr_nom / Rr

    # ============================================================
    # Actual physical plant
    # ============================================================
    plant_p = IMPlantParams(
        Rs = Rs * plant_Rs_scale,
        Rr = Rr * plant_Rr_scale,
        Lls = Lls_nom * plant_Lls_scale,
        Llr = Llr_nom * plant_Llr_scale,
        Lm = Lm_nom * plant_Lm_scale,
        p = p_pairs,
        J = J * plant_J_scale,
        B = B * plant_B_scale,
    )

    # ============================================================
    # Rotor-flux observer assumed model
    # ============================================================
    obs_p = RotorFluxObserverDiscreteParams(
        Lm = Lm_nom * obs_Lm_scale,
        Lss = Lss_nom * obs_Lss_scale,
        Lrr = Lrr_nom * obs_Lrr_scale,
        tau_r = tau_r_nom * obs_tau_r_scale,
        p = p_pairs,
        Ts = Ts,
        tau_Te_speed = obs_tau_Te_speed,
        tau_Te_torque = obs_tau_Te_torque,
    )

    # ============================================================
    # Load-torque estimator assumed model
    # ============================================================
    load_kalman_p = LoadTorqueKalmanParams(
        J = J * load_est_J_scale,
        B = B * load_est_B_scale,
        Ts = Ts,
        R = TL_kalman_R,
        q_omega = TL_kalman_q_omega,
        q_TL = TL_kalman_q_TL,
        limit_positive = TL_kalman_limit_positive,
    )

    # ============================================================
    # F1 outer speed/flux controller assumed model
    # ============================================================
    outer_p = OuterSpeedFluxF1Params(
        Ts = Ts,

        p = p_pairs,
        Lm = Lm_nom * outer_Lm_scale,
        Lrr = Lrr_nom * outer_Lrr_scale,
        J = J * outer_J_scale,
        B = B * outer_B_scale,

        isd_nom = isd_nom,

        Is_max = outer_Is_max,
        isd_min = isd_min,
        Te_max = Te_max,
        wm_dot_max = wm_dot_max,
        id_dot_max = id_dot_max,

        use_field_weakening = use_field_weakening,
        wm_base_fw = wm_base_fw,

        tau_f_wm = outer_tau_f_wm,
        ts_wm = outer_ts_wm,
        ts_dist_wm = outer_ts_dist_wm,

        use_load_feedforward = use_load_feedforward,
        load_ff_sign = load_ff_sign,
    )

    # ============================================================
    # Inner current controller assumed model
    # ============================================================
    ctrl_p = CurrentControllerDiscreteParams(
        Rs = Rs * ctrl_Rs_scale,
        sigma_Lss = sigma_Lss_nom * ctrl_sigma_Lss_scale,
        k = k_nom * ctrl_k_scale,

        Ts = Ts,
        ts_spec = current_ts_spec,
        tau_f = current_tau_f,

        Vs_max = Vs_max,
        Is_max = Is_max,

        use_filter = use_filter,
        use_feedforward = use_feedforward,
        use_saturation = use_saturation,
        use_antiwindup = use_antiwindup,
    )

    # ============================================================
    # Initial states
    # ============================================================
    x = IMPlantState()

    obs_state = RotorFluxObserverDiscreteState()
    load_kalman_state = LoadTorqueKalmanState()

    # Start the F1 d-axis reference at the configured nominal value.
    outer_state = OuterSpeedFluxF1State(
        id_ref_ant = isd_nom,
    )

    ctrl_state = CurrentControllerDiscreteState()

    N = Int(floor(t_end / Ts)) + 1
    h = Ts / plant_substeps

    # ============================================================
    # Preallocate result vectors
    # ============================================================
    time = Vector{Float64}(undef, N)

    wm_ref_vec = Vector{Float64}(undef, N)
    wm_ref_ramp_vec = Vector{Float64}(undef, N)
    wm_filt_vec = Vector{Float64}(undef, N)
    e_wm_vec = Vector{Float64}(undef, N)

    Te_ref_out_vec = Vector{Float64}(undef, N)
    Te_PI_vec = Vector{Float64}(undef, N)
    Te_ff_vec = Vector{Float64}(undef, N)

    isd_ref_vec = Vector{Float64}(undef, N)
    isq_ref_vec = Vector{Float64}(undef, N)
    isd_vec = Vector{Float64}(undef, N)
    isq_vec = Vector{Float64}(undef, N)
    isd_lim_vec = Vector{Float64}(undef, N)
    isq_lim_vec = Vector{Float64}(undef, N)

    vsd_vec = Vector{Float64}(undef, N)
    vsq_vec = Vector{Float64}(undef, N)
    vsα_vec = Vector{Float64}(undef, N)
    vsβ_vec = Vector{Float64}(undef, N)
    vs_mod_vec = Vector{Float64}(undef, N)
    vs_mod_unsat_vec = Vector{Float64}(undef, N)

    omega_vec = Vector{Float64}(undef, N)
    speed_rpm_vec = Vector{Float64}(undef, N)

    torque_vec = Vector{Float64}(undef, N)
    torque_obs_vec = Vector{Float64}(undef, N)

    flux_r_vec = Vector{Float64}(undef, N)
    lambda_ref_vec = Vector{Float64}(undef, N)
    theta_e_vec = Vector{Float64}(undef, N)
    omega_e_vec = Vector{Float64}(undef, N)

    Tload_vec = Vector{Float64}(undef, N)
    TL_est_vec = Vector{Float64}(undef, N)
    omega_hat_load_vec = Vector{Float64}(undef, N)

    ia_vec = Vector{Float64}(undef, N)
    ib_vec = Vector{Float64}(undef, N)
    ic_vec = Vector{Float64}(undef, N)

    va_vec = Vector{Float64}(undef, N)
    vb_vec = Vector{Float64}(undef, N)
    vc_vec = Vector{Float64}(undef, N)

    Pelec_vec = Vector{Float64}(undef, N)
    Pmech_vec = Vector{Float64}(undef, N)
    Pload_vec = Vector{Float64}(undef, N)
    Pfric_vec = Vector{Float64}(undef, N)

    sat_current_vec = Vector{Float64}(undef, N)
    sat_isd_vec = Vector{Float64}(undef, N)
    sat_isq_vec = Vector{Float64}(undef, N)
    sat_Te_vec = Vector{Float64}(undef, N)
    field_weakening_vec = Vector{Float64}(undef, N)

    # ============================================================
    # Main sampled-data simulation loop
    # ============================================================
    for kstep in 1:N
        t = (kstep - 1) * Ts

        # --------------------------------------------------------
        # Reference and disturbance profiles
        # --------------------------------------------------------
        wm_ref = im_160kw_speed_reference(
            t;
            wm_abs = wm_ref_abs,
            t_positive_ramp_start = t_positive_ramp_start,
            t_positive_ramp_end = t_positive_ramp_end,
            t_positive_hold_end = t_positive_hold_end,
            t_positive_return_end = t_positive_return_end,
            t_negative_ramp_start = t_negative_ramp_start,
            t_negative_ramp_end = t_negative_ramp_end,
            t_negative_hold_end = t_negative_hold_end,
            t_negative_return_end = t_negative_return_end,
        )

        Tload_k = im_160kw_load_torque_profile(
            t;
            Tload_step1 = Tload_step1,
            Tload_step2 = Tload_step2,
            t_positive_step1 = t_positive_load_step1,
            t_positive_step2 = t_positive_load_step2,
            t_positive_release = t_positive_load_release,
            t_negative_step1 = t_negative_load_step1,
            t_negative_step2 = t_negative_load_step2,
            t_negative_release = t_negative_load_release,
        )

        # --------------------------------------------------------
        # Rotor-flux and torque observer
        # --------------------------------------------------------
        obs = rotor_flux_observer_step!(
            obs_state,
            obs_p;
            i_alpha = x.isα,
            i_beta = x.isβ,
            theta_m = x.θm,
            omega_m = x.ωm,
            reset = false,
        )

        # --------------------------------------------------------
        # Load-torque estimate for the outer-loop feedforward
        # --------------------------------------------------------
        if load_estimator == :none
            TL_est = 0.0
            omega_hat_load = x.ωm

        elseif load_estimator == :actual
            TL_est = Tload_k
            omega_hat_load = x.ωm

        else
            load_obs = load_torque_kalman_step!(
                load_kalman_state,
                load_kalman_p;
                omega = x.ωm,
                Te = obs.Te_raw,
                reset = false,
            )

            TL_est = load_obs.TL_hat
            omega_hat_load = load_obs.omega_hat
        end

        # --------------------------------------------------------
        # F1 outer speed/flux controller
        # --------------------------------------------------------
        outer = outer_speed_flux_f1_step!(
            outer_state,
            outer_p;
            wm_ref = wm_ref,
            wm_med = x.ωm,
            TL_est = TL_est,
            reset = false,
        )

        # --------------------------------------------------------
        # Inner dq current controller
        # --------------------------------------------------------
        ctrl = current_controller_step!(
            ctrl_state,
            ctrl_p;
            isd_ref = outer.isd_ref,
            isq_ref = outer.isq_ref,
            isd_med = obs.i_sd_e,
            isq_med = obs.i_sq_e,
            omega_e = obs.omega_e,
            lambda_rd = obs.lambda_rd_e,
            reset = false,
        )

        # --------------------------------------------------------
        # dq voltage -> stationary alpha-beta voltage
        # --------------------------------------------------------
        vsα_hold, vsβ_hold = inverse_park_voltage(
            ctrl.vsd,
            ctrl.vsq,
            obs.theta_e,
        )

        vs_mod = hypot(ctrl.vsd, ctrl.vsq)

        # --------------------------------------------------------
        # Reconstruct balanced abc quantities
        # --------------------------------------------------------
        ia_k = x.isα
        ib_k = -0.5 * x.isα + sqrt(3.0) / 2.0 * x.isβ
        ic_k = -0.5 * x.isα - sqrt(3.0) / 2.0 * x.isβ

        va_k = vsα_hold
        vb_k = -0.5 * vsα_hold + sqrt(3.0) / 2.0 * vsβ_hold
        vc_k = -0.5 * vsα_hold - sqrt(3.0) / 2.0 * vsβ_hold

        # --------------------------------------------------------
        # Torque and power before the plant update
        # --------------------------------------------------------
        torque_k = im_torque(
            x,
            plant_p,
        )

        Pelec_k =
            va_k * ia_k +
            vb_k * ib_k +
            vc_k * ic_k

        Pmech_k = torque_k * x.ωm
        Pload_k = Tload_k * x.ωm
        Pfric_k = plant_p.B * x.ωm^2

        # --------------------------------------------------------
        # Store signals
        # --------------------------------------------------------
        time[kstep] = t

        wm_ref_vec[kstep] = wm_ref
        wm_ref_ramp_vec[kstep] = outer.wm_ref_ramp
        wm_filt_vec[kstep] = outer.wm_filt
        e_wm_vec[kstep] = outer.e_wm

        Te_ref_out_vec[kstep] = outer.Te_ref_out
        Te_PI_vec[kstep] = outer.Te_PI
        Te_ff_vec[kstep] = outer.Te_ff

        isd_ref_vec[kstep] = outer.isd_ref
        isq_ref_vec[kstep] = outer.isq_ref
        isd_vec[kstep] = obs.i_sd_e
        isq_vec[kstep] = obs.i_sq_e
        isd_lim_vec[kstep] = ctrl.isd_ref_lim
        isq_lim_vec[kstep] = ctrl.isq_ref_lim

        vsd_vec[kstep] = ctrl.vsd
        vsq_vec[kstep] = ctrl.vsq
        vsα_vec[kstep] = vsα_hold
        vsβ_vec[kstep] = vsβ_hold
        vs_mod_vec[kstep] = vs_mod
        vs_mod_unsat_vec[kstep] = ctrl.vs_mod_unsat

        omega_vec[kstep] = x.ωm
        speed_rpm_vec[kstep] = x.ωm * 60.0 / (2π)

        torque_vec[kstep] = torque_k
        torque_obs_vec[kstep] = obs.Te_raw

        flux_r_vec[kstep] = obs.flux_r_mod
        lambda_ref_vec[kstep] = outer.lambda_ref_out
        theta_e_vec[kstep] = obs.theta_e
        omega_e_vec[kstep] = obs.omega_e

        Tload_vec[kstep] = Tload_k
        TL_est_vec[kstep] = TL_est
        omega_hat_load_vec[kstep] = omega_hat_load

        ia_vec[kstep] = ia_k
        ib_vec[kstep] = ib_k
        ic_vec[kstep] = ic_k

        va_vec[kstep] = va_k
        vb_vec[kstep] = vb_k
        vc_vec[kstep] = vc_k

        Pelec_vec[kstep] = Pelec_k
        Pmech_vec[kstep] = Pmech_k
        Pload_vec[kstep] = Pload_k
        Pfric_vec[kstep] = Pfric_k

        sat_current_vec[kstep] = ctrl.saturado ? 1.0 : 0.0
        sat_isd_vec[kstep] = Float64(outer.sat_isd)
        sat_isq_vec[kstep] = Float64(outer.sat_isq)
        sat_Te_vec[kstep] = Float64(outer.sat_Te)
        field_weakening_vec[kstep] =
            Float64(outer.field_weakening_active)

        # --------------------------------------------------------
        # Continuous alpha-beta plant integration
        # --------------------------------------------------------
        for _ in 1:plant_substeps
            x = rk4_step(
                x,
                plant_p,
                h;
                vsα = vsα_hold,
                vsβ = vsβ_hold,
                Tload = Tload_k,
            )
        end
    end

    return (
        t = time,

        wm_ref = wm_ref_vec,
        wm_ref_ramp = wm_ref_ramp_vec,
        wm_filt = wm_filt_vec,
        e_wm = e_wm_vec,
        omega_m = omega_vec,
        speed_rpm = speed_rpm_vec,

        Te_ref_out = Te_ref_out_vec,
        Te_PI = Te_PI_vec,
        Te_ff = Te_ff_vec,
        torque = torque_vec,
        torque_obs = torque_obs_vec,
        Tload = Tload_vec,
        TL_est = TL_est_vec,
        omega_hat_load = omega_hat_load_vec,

        isd_ref = isd_ref_vec,
        isq_ref = isq_ref_vec,
        isd = isd_vec,
        isq = isq_vec,
        isd_ref_lim = isd_lim_vec,
        isq_ref_lim = isq_lim_vec,

        vsd = vsd_vec,
        vsq = vsq_vec,
        vsα = vsα_vec,
        vsβ = vsβ_vec,
        vs_mod = vs_mod_vec,
        vs_mod_unsat = vs_mod_unsat_vec,

        flux_r = flux_r_vec,
        lambda_ref = lambda_ref_vec,
        theta_e = theta_e_vec,
        omega_e = omega_e_vec,

        ia = ia_vec,
        ib = ib_vec,
        ic = ic_vec,
        va = va_vec,
        vb = vb_vec,
        vc = vc_vec,

        Pelec = Pelec_vec,
        Pmech = Pmech_vec,
        Pload = Pload_vec,
        Pfric = Pfric_vec,

        saturation_current = sat_current_vec,
        sat_isd = sat_isd_vec,
        sat_isq = sat_isq_vec,
        sat_Te = sat_Te_vec,
        field_weakening_active = field_weakening_vec,

        # Return the parameter objects and nominal derived values to make
        # each simulation fully inspectable and reproducible.
        plant_p = plant_p,
        obs_p = obs_p,
        outer_p = outer_p,
        ctrl_p = ctrl_p,
        load_kalman_p = load_kalman_p,

        nominal = (
            fn = fn,
            Rs = Rs,
            Rr = Rr,
            Lls = Lls_nom,
            Llr = Llr_nom,
            Lm = Lm_nom,
            Lss = Lss_nom,
            Lrr = Lrr_nom,
            sigma_Lss = sigma_Lss_nom,
            k = k_nom,
            tau_r = tau_r_nom,
            p_pairs = p_pairs,
            J = J,
            B = B,
            Vdc = Vdc,
            Vs_max = Vs_max,
            Is_max = Is_max,
            outer_Is_max = outer_Is_max,
            Te_max = Te_max,
            isd_nom = isd_nom,
            wm_base_fw = wm_base_fw,
        ),
    )
end
