# ============================================================
# Hybrid FOC torque-control simulator: outer torque loop F1 + inner current loop
# ============================================================
#
# Discrete blocks:
#   outer_torque_flux_f1_step!
#   rotor_flux_observer_step!
#   current_controller_step!
#
# Continuous plant:
#   IM alpha-beta model integrated with RK4
#
# Control architecture:
#
#   Te_ref profile
#        ↓
#   outer torque/flux F1
#        ↓
#   isd_ref, isq_ref
#        ↓
#   discrete current controller
#        ↓
#   vsd, vsq
#        ↓
#   inverse Park
#        ↓
#   continuous IM plant
# ============================================================

function torque_reference_steps(
    t;
    t_Te_pos_step = 2.0,
    t_Te_neg_step = 6.0,
    t_Te_zero_step = 10.0,
    Te_ref_pos = 20.0,
    Te_ref_neg = -20.0,
)

    if t < t_Te_pos_step
        return 0.0
    elseif t < t_Te_neg_step
        return Te_ref_pos
    elseif t < t_Te_zero_step
        return Te_ref_neg
    else
        return 0.0
    end
end

function simulate_foc_torque_f1_hybrid(;
    t_end = 12.0,
    Ts = 100e-6,
    plant_substeps = 1,

    # Torque reference profile
    t_Te_pos_step = 2.0,
    t_Te_neg_step = 6.0,
    t_Te_zero_step = 10.0,
    Te_ref_pos = 20.0,
    Te_ref_neg = -20.0,

    # Load
    load_profile::Symbol = :constant,
    Tload = 0.0,
    Tload_step1 = 40.0,
    Tload_step2 = 80.0,
    t_load_step1 = 6.0,
    t_load_step2 = 8.0,

    # Machine nominal parameters
    Rs = 0.21946,
    Rr = 0.11659,
    Lls = 4.28e-3,
    Llr = 4.28e-3,
    Lm = 40.84e-3,
    p_pairs = 2.0,
    J = 0.2685 + 0.1,
    B = 0.01298,

    # Outer-loop parameters
    isd_nom = 23.04579328,
    outer_Is_max = 40.0,
    isd_min = 5.0,
    Te_max = 124.0419647,
    Te_dot_max = 500.0,
    id_dot_max = 600.0,

    # Inner current-controller limits
    Vs_max = 310.0,
    Is_max = 40.0,

    # Current-controller options
    use_filter = true,
    use_feedforward = true,
    use_saturation = true,
    use_antiwindup = true,

    # Optional parameter mismatch multipliers
    plant_Rs_scale = 1.0,
    plant_Rr_scale = 1.0,
    plant_Lls_scale = 1.0,
    plant_Llr_scale = 1.0,
    plant_Lm_scale = 1.0,
    plant_J_scale = 1.0,
    plant_B_scale = 1.0,

    obs_Lm_scale = 1.0,
    obs_Lss_scale = 1.0,
    obs_Lrr_scale = 1.0,
    obs_tau_r_scale = 1.0,

    ctrl_Rs_scale = 1.0,
    ctrl_sigma_Lss_scale = 1.0,
    ctrl_k_scale = 1.0,
)

    # ============================================================
    # Nominal parameters
    # ============================================================

    Rs_nom = Rs
    Rr_nom = Rr
    Lls_nom = Lls
    Llr_nom = Llr
    Lm_nom = Lm
    J_nom = J
    B_nom = B

    Lss_nom = Lls_nom + Lm_nom
    Lrr_nom = Llr_nom + Lm_nom
    sigma_Lss_nom = Lss_nom - Lm_nom^2 / Lrr_nom
    k_nom = Lm_nom / Lrr_nom
    tau_r_nom = Lrr_nom / Rr_nom

    # ============================================================
    # Actual plant parameters
    # ============================================================

    plant_p = IMPlantParams(
        Rs = Rs_nom * plant_Rs_scale,
        Rr = Rr_nom * plant_Rr_scale,
        Lls = Lls_nom * plant_Lls_scale,
        Llr = Llr_nom * plant_Llr_scale,
        Lm = Lm_nom * plant_Lm_scale,
        p = p_pairs,
        J = J_nom * plant_J_scale,
        B = B_nom * plant_B_scale,
    )

    # ============================================================
    # Observer assumed parameters
    # ============================================================

    obs_p = RotorFluxObserverDiscreteParams(
        Lm = Lm_nom * obs_Lm_scale,
        Lss = Lss_nom * obs_Lss_scale,
        Lrr = Lrr_nom * obs_Lrr_scale,
        tau_r = tau_r_nom * obs_tau_r_scale,
        p = p_pairs,
        Ts = Ts,
    )

    # ============================================================
    # Outer-loop assumed parameters
    # ============================================================

    outer_p = OuterTorqueFluxF1Params(
        Ts = Ts,
        p = p_pairs,
        Lm = Lm_nom,
        Lrr = Lrr_nom,
        isd_nom = isd_nom,
        Is_max = outer_Is_max,
        isd_min = isd_min,
        Te_max = Te_max,
        Te_dot_max = Te_dot_max,
        id_dot_max = id_dot_max,
    )

    # ============================================================
    # Current-controller assumed parameters
    # ============================================================

    ctrl_p = CurrentControllerDiscreteParams(
        Rs = Rs_nom * ctrl_Rs_scale,
        sigma_Lss = sigma_Lss_nom * ctrl_sigma_Lss_scale,
        k = k_nom * ctrl_k_scale,
        Ts = Ts,
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
    outer_state = OuterTorqueFluxF1State(id_ref_ant = isd_nom)
    ctrl_state = CurrentControllerDiscreteState()

    N = Int(floor(t_end / Ts)) + 1
    h = Ts / plant_substeps

    # ============================================================
    # Preallocate result vectors
    # ============================================================

    time = Vector{Float64}(undef, N)

    Te_ref_ext_vec = Vector{Float64}(undef, N)
    Te_ref_out_vec = Vector{Float64}(undef, N)

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

    speed_vec = Vector{Float64}(undef, N)
    omega_vec = Vector{Float64}(undef, N)

    torque_vec = Vector{Float64}(undef, N)
    torque_obs_vec = Vector{Float64}(undef, N)

    flux_r_vec = Vector{Float64}(undef, N)
    theta_e_vec = Vector{Float64}(undef, N)
    omega_e_vec = Vector{Float64}(undef, N)

    Tload_vec = Vector{Float64}(undef, N)

    isα_vec = Vector{Float64}(undef, N)
    isβ_vec = Vector{Float64}(undef, N)

    sat_current_vec = Vector{Float64}(undef, N)
    sat_isd_vec = Vector{Float64}(undef, N)
    sat_isq_vec = Vector{Float64}(undef, N)
    sat_Te_vec = Vector{Float64}(undef, N)

    vs_mod_unsat_vec = Vector{Float64}(undef, N)

    # ============================================================
    # Main hybrid simulation loop
    # ============================================================

    for kstep in 1:N
        t = (kstep - 1) * Ts

        # --------------------------------------------------------
        # External torque reference and load
        # --------------------------------------------------------

        Te_ref_ext = torque_reference_steps(
            t;
            t_Te_pos_step = t_Te_pos_step,
            t_Te_neg_step = t_Te_neg_step,
            t_Te_zero_step = t_Te_zero_step,
            Te_ref_pos = Te_ref_pos,
            Te_ref_neg = Te_ref_neg,
        )

        Tload_k = load_torque_profile(
            t;
            load_profile = load_profile,
            Tload = Tload,
            Tload_step1 = Tload_step1,
            Tload_step2 = Tload_step2,
            t_load_step1 = t_load_step1,
            t_load_step2 = t_load_step2,
        )

        # --------------------------------------------------------
        # Observer update
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
        # Outer torque/flux loop
        # --------------------------------------------------------

        outer = outer_torque_flux_f1_step!(
            outer_state,
            outer_p;
            Te_ref_ext = Te_ref_ext,
            TL = Tload_k,
            reset = false,
        )

        # --------------------------------------------------------
        # Inner current controller
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
        # Voltage command dq -> alpha-beta
        # --------------------------------------------------------

        vsα_hold, vsβ_hold = inverse_park_voltage(ctrl.vsd, ctrl.vsq, obs.theta_e)

        # --------------------------------------------------------
        # Store values before plant update
        # --------------------------------------------------------

        time[kstep] = t

        Te_ref_ext_vec[kstep] = Te_ref_ext
        Te_ref_out_vec[kstep] = outer.Te_ref_out

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

        speed_vec[kstep] = x.ωm * 60 / (2π)
        omega_vec[kstep] = x.ωm

        torque_vec[kstep] = im_torque(x, plant_p)
        torque_obs_vec[kstep] = obs.Te_raw

        flux_r_vec[kstep] = obs.flux_r_mod
        theta_e_vec[kstep] = obs.theta_e
        omega_e_vec[kstep] = obs.omega_e

        Tload_vec[kstep] = Tload_k

        isα_vec[kstep] = x.isα
        isβ_vec[kstep] = x.isβ

        sat_current_vec[kstep] = ctrl.saturado ? 1.0 : 0.0
        sat_isd_vec[kstep] = outer.sat_isd
        sat_isq_vec[kstep] = outer.sat_isq
        sat_Te_vec[kstep] = outer.sat_Te

        vs_mod_unsat_vec[kstep] = ctrl.vs_mod_unsat

        # --------------------------------------------------------
        # Plant integration over one controller sample
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

        Te_ref_ext = Te_ref_ext_vec,
        Te_ref_out = Te_ref_out_vec,

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

        speed_rpm = speed_vec,
        omega_m = omega_vec,

        torque = torque_vec,
        torque_obs = torque_obs_vec,

        flux_r = flux_r_vec,
        theta_e = theta_e_vec,
        omega_e = omega_e_vec,

        Tload = Tload_vec,

        isα = isα_vec,
        isβ = isβ_vec,

        saturation_current = sat_current_vec,
        sat_isd = sat_isd_vec,
        sat_isq = sat_isq_vec,
        sat_Te = sat_Te_vec,

        vs_mod_unsat = vs_mod_unsat_vec,
    )
end