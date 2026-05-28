"""
    build_foc_current_im_system(; kwargs...)

Builds a first current-loop FOC test system.

Connections:

    current reference profile -> FOC current controller -> IM plant
    load profile ---------------------------------------> IM plant
    IM plant outputs -----------------------------------> rotor-flux observer
    observer outputs -----------------------------------> FOC current controller

This is not yet full FOC. It is only the inner current-loop test.
"""
function build_foc_current_im_system(;
    # ------------------------------------------------------------
    # Machine parameters
    # ------------------------------------------------------------
    Rs_val = 0.21946,
    Rr_val = 0.11659,
    Lls_val = 4.28e-3,
    Llr_val = 4.28e-3,
    Lm_val = 40.84e-3,
    p_val = 2.0,
    J_val = 0.2685 + 0.1,
    B_val = 0.01298,

    # ------------------------------------------------------------
    # Current controller parameters
    # ------------------------------------------------------------
    Vs_max_val = 310.0,
    Is_max_val = 40.0,
    current_loop_tau_val = 2e-3 / 3,

    # ------------------------------------------------------------
    # Current reference profile
    # ------------------------------------------------------------
    current_reference_profile = :steps,
    t_id_step_val = 1.0,
    t_iq_pos_step_val = 3.0,
    t_iq_neg_step_val = 6.0,
    t_iq_zero_step_val = 9.0,

    isd_ref_mag_val = 10.0,
    isq_ref_pos_val = 15.0,
    isq_ref_neg_val = -15.0,

    # ------------------------------------------------------------
    # Load torque parameters
    # ------------------------------------------------------------
    Tload_val = 0.0,

    load_profile = :constant,
    Tload_step1_val = 40.0,
    Tload_step2_val = 80.0,
    t_load_step1_val = 6.0,
    t_load_step2_val = 8.0,

    # ------------------------------------------------------------
    # Observer parameters
    # ------------------------------------------------------------
    tau_r_val = 0.3869971696,
)

    # ============================================================
    # Parameters
    # ============================================================

    @parameters begin
        Rs = Rs_val
        Rr = Rr_val
        Lls = Lls_val
        Llr = Llr_val
        Lm = Lm_val
        p = p_val
        J = J_val
        B = B_val

        Tload = Tload_val

        Lss = Lls_val + Lm_val
        Lrr = Llr_val + Lm_val
        tau_r = tau_r_val

        Vs_max = Vs_max_val
        Is_max = Is_max_val
        current_loop_tau = current_loop_tau_val

        # For the current controller, this is the transient inductance.
        # Equivalent to sigma * Lss = Lss - Lm^2/Lrr.
        sigma_Lss = (Lls_val + Lm_val) - (Lm_val^2 / (Llr_val + Lm_val))
    end

    pars = (
        Rs = Rs,
        Rr = Rr,
        Lls = Lls,
        Llr = Llr,
        Lm = Lm,
        p = p,
        J = J,
        B = B,

        Tload = Tload,

        Lss = Lss,
        Lrr = Lrr,
        tau_r = tau_r,

        Vs_max = Vs_max,
        Is_max = Is_max,
        current_loop_tau = current_loop_tau,
        sigma_Lss = sigma_Lss,
    )

    # ============================================================
    # Variables
    # ============================================================

    @variables begin
        # --------------------------------------------------------
        # Current reference profile variables
        # --------------------------------------------------------
        isd_ref(t)
        isq_ref(t)

        # --------------------------------------------------------
        # Load torque profile variable
        # --------------------------------------------------------
        Tload_cmd(t)

        # --------------------------------------------------------
        # FOC current controller variables
        # --------------------------------------------------------
        ui_d(t) = 0.0
        ui_q(t) = 0.0

        isd_ref_lim(t)
        isq_ref_lim(t)
        isq_max_available(t)

        err_d(t)
        err_q(t)

        vsd_PI(t)
        vsq_PI(t)

        vsd_ff(t)
        vsq_ff(t)

        vsd_unsat(t)
        vsq_unsat(t)

        vs_mod_unsat(t)
        voltage_scale(t)
        voltage_saturation(t)

        vsd_ref(t)
        vsq_ref(t)

        is_mod_obs(t)

        # --------------------------------------------------------
        # Controller output / plant input voltages
        # --------------------------------------------------------
        vsα(t)
        vsβ(t)

        # Dummy variables kept for compatibility with plots/scripts if needed
        va(t)
        vb(t)
        vc(t)

        # --------------------------------------------------------
        # Induction machine plant variables
        # --------------------------------------------------------
        isα(t) = 0.0
        isβ(t) = 0.0

        irα(t) = 0.0
        irβ(t) = 0.0

        ψsα(t)
        ψsβ(t)

        ψrα(t)
        ψrβ(t)

        ωm(t) = 0.0
        θm(t) = 0.0

        Te(t)
        n_rpm(t)
        n_sync_rpm(t)

        # --------------------------------------------------------
        # Rotor-flux and torque observer variables
        # --------------------------------------------------------
        θr_obs(t)

        isd_r_obs(t)
        isq_r_obs(t)

        λrd_r_obs(t) = 0.0
        λrq_r_obs(t) = 0.0

        λrα_obs(t)
        λrβ_obs(t)

        flux_r_mod_obs(t)
        θe_obs(t)

        isd_e_obs(t)
        isq_e_obs(t)

        λrd_e_obs(t)
        λrq_e_obs(t)

        Te_obs(t)

        ψsd_e_obs(t)
        ψsq_e_obs(t)
        flux_s_mod_obs(t)

        wsl_obs(t)
        ws_obs(t)

        # For compatibility with plant output n_sync_rpm.
        # In FOC this is an estimated synchronous speed output, not a command.
        f_cmd(t)
    end

    vars = (
        # Reference/profile variables
        isd_ref = isd_ref,
        isq_ref = isq_ref,
        Tload_cmd = Tload_cmd,

        # Current controller variables
        ui_d = ui_d,
        ui_q = ui_q,

        isd_ref_lim = isd_ref_lim,
        isq_ref_lim = isq_ref_lim,
        isq_max_available = isq_max_available,

        err_d = err_d,
        err_q = err_q,

        vsd_PI = vsd_PI,
        vsq_PI = vsq_PI,

        vsd_ff = vsd_ff,
        vsq_ff = vsq_ff,

        vsd_unsat = vsd_unsat,
        vsq_unsat = vsq_unsat,

        vs_mod_unsat = vs_mod_unsat,
        voltage_scale = voltage_scale,
        voltage_saturation = voltage_saturation,

        vsd_ref = vsd_ref,
        vsq_ref = vsq_ref,

        is_mod_obs = is_mod_obs,

        # Plant input voltages
        vsα = vsα,
        vsβ = vsβ,

        va = va,
        vb = vb,
        vc = vc,

        # Plant variables
        isα = isα,
        isβ = isβ,

        irα = irα,
        irβ = irβ,

        ψsα = ψsα,
        ψsβ = ψsβ,

        ψrα = ψrα,
        ψrβ = ψrβ,

        ωm = ωm,
        θm = θm,

        Te = Te,
        n_rpm = n_rpm,
        n_sync_rpm = n_sync_rpm,

        # Observer variables
        θr_obs = θr_obs,

        isd_r_obs = isd_r_obs,
        isq_r_obs = isq_r_obs,

        λrd_r_obs = λrd_r_obs,
        λrq_r_obs = λrq_r_obs,

        λrα_obs = λrα_obs,
        λrβ_obs = λrβ_obs,

        flux_r_mod_obs = flux_r_mod_obs,
        θe_obs = θe_obs,

        isd_e_obs = isd_e_obs,
        isq_e_obs = isq_e_obs,

        λrd_e_obs = λrd_e_obs,
        λrq_e_obs = λrq_e_obs,

        Te_obs = Te_obs,

        ψsd_e_obs = ψsd_e_obs,
        ψsq_e_obs = ψsq_e_obs,
        flux_s_mod_obs = flux_s_mod_obs,

        wsl_obs = wsl_obs,
        ws_obs = ws_obs,

        f_cmd = f_cmd,
    )

    # ============================================================
    # Current reference profile
    # ============================================================

    if current_reference_profile == :steps

        isd_ref_expr = ifelse(
            t < t_id_step_val,
            0.0,
            isd_ref_mag_val,
        )

        isq_ref_expr = ifelse(
            t < t_iq_pos_step_val,
            0.0,
            ifelse(
                t < t_iq_neg_step_val,
                isq_ref_pos_val,
                ifelse(
                    t < t_iq_zero_step_val,
                    isq_ref_neg_val,
                    0.0,
                ),
            ),
        )

    elseif current_reference_profile == :constant

        isd_ref_expr = isd_ref_mag_val
        isq_ref_expr = 0.0

    else
        error("Unknown current_reference_profile = $current_reference_profile. Use :steps or :constant.")
    end

    current_ref_eqs = [
        isd_ref ~ isd_ref_expr,
        isq_ref ~ isq_ref_expr,
    ]

    # ============================================================
    # Load profile
    # ============================================================

    load_eqs = build_load_torque_eqs(
        Tload_cmd;
        load_profile = load_profile,
        Tload = Tload,
        Tload_step1_val = Tload_step1_val,
        Tload_step2_val = Tload_step2_val,
        t_load_step1_val = t_load_step1_val,
        t_load_step2_val = t_load_step2_val,
    )

    # ============================================================
    # Observer
    # ============================================================

    observer_eqs = build_rotor_flux_observer_eqs(vars, pars)

    # ============================================================
    # Current controller
    # ============================================================

    control_eqs = build_foc_current_controller_eqs(vars, pars)

    # ============================================================
    # Plant
    # ============================================================

    # In FOC, f_cmd is not a command. It is only used by the current plant
    # function to compute n_sync_rpm. We define it from the estimated
    # synchronous electrical speed for compatibility.
    speed_aux_eqs = [
        f_cmd ~ vars.ws_obs / (2π),
    ]

    plant_eqs = build_induction_machine_alpha_beta_eqs(vars, pars)

    # Dummy abc voltages for compatibility, not used by the plant here.
    dummy_abc_eqs = [
        va ~ 0.0,
        vb ~ 0.0,
        vc ~ 0.0,
    ]

    # ============================================================
    # Full system
    # ============================================================

    eqs = vcat(
        current_ref_eqs,
        load_eqs,
        observer_eqs,
        control_eqs,
        speed_aux_eqs,
        plant_eqs,
        dummy_abc_eqs,
    )

    @named sys = ODESystem(eqs, t)
    return structural_simplify(sys)
end


"""
    simulate_foc_current_im(; kwargs...)

Builds and simulates the current-loop FOC induction-machine system.
"""
function simulate_foc_current_im(;
    tspan = (0.0, 12.0),

    current_reference_profile = :steps,
    t_id_step_val = 1.0,
    t_iq_pos_step_val = 3.0,
    t_iq_neg_step_val = 6.0,
    t_iq_zero_step_val = 9.0,

    isd_ref_mag_val = 10.0,
    isq_ref_pos_val = 15.0,
    isq_ref_neg_val = -15.0,

    load_profile = :constant,
    Tload_step1_val = 40.0,
    Tload_step2_val = 80.0,
    t_load_step1_val = 6.0,
    t_load_step2_val = 8.0,

    solver = Rodas5P(),
    reltol = 1e-6,
    abstol = 1e-8,

    kwargs...
)

    sys = build_foc_current_im_system(;
        current_reference_profile = current_reference_profile,
        t_id_step_val = t_id_step_val,
        t_iq_pos_step_val = t_iq_pos_step_val,
        t_iq_neg_step_val = t_iq_neg_step_val,
        t_iq_zero_step_val = t_iq_zero_step_val,

        isd_ref_mag_val = isd_ref_mag_val,
        isq_ref_pos_val = isq_ref_pos_val,
        isq_ref_neg_val = isq_ref_neg_val,

        load_profile = load_profile,
        Tload_step1_val = Tload_step1_val,
        Tload_step2_val = Tload_step2_val,
        t_load_step1_val = t_load_step1_val,
        t_load_step2_val = t_load_step2_val,

        kwargs...
    )

    prob = ODEProblem(sys, [], tspan)

    tstops = Float64[]

    if current_reference_profile == :steps
        append!(tstops, [
            t_id_step_val,
            t_iq_pos_step_val,
            t_iq_neg_step_val,
            t_iq_zero_step_val,
        ])
    end

    append!(
        tstops,
        load_profile_tstops(
            load_profile = load_profile,
            t_load_step1_val = t_load_step1_val,
            t_load_step2_val = t_load_step2_val,
            tspan = tspan,
        ),
    )

    tstops = unique(sort(filter(τ -> tspan[1] < τ < tspan[2], tstops)))

    if isempty(tstops)
        sol = solve(prob, solver; reltol = reltol, abstol = abstol)
    else
        sol = solve(prob, solver; reltol = reltol, abstol = abstol, tstops = tstops)
    end

    return sol, sys
end