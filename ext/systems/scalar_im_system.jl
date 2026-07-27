"""
    build_scalar_im_system(; kwargs...)

Builds the complete scalar-control induction-machine system.

This is the equivalent of the top-level Simulink diagram.

Connections:

    frequency profile -> scalar V/f controller -> IM plant
    load profile -------------------------------> IM plant
    IM plant outputs ---------------------------> optional observer
"""
function build_scalar_im_system(;
    # Machine parameters
    Rs_val = 0.21946,
    Rr_val = 0.11659,
    Lls_val = 4.28e-3,
    Llr_val = 4.28e-3,
    Lm_val = 40.84e-3,
    p_val = 2.0,
    J_val = 0.2685 + 0.1,
    B_val = 0.01298,

    # Scalar-control parameters
    Vll_nom_val = 380.0,
    f_nom_val = 50.0,
    f_ref_val = 25.0,

    frequency_profile = :constant,
    f_step_val = 5.0,
    t_hold_val = 1.0,
    t_step_val = 1.0,

    # Mechanical load parameters
    Tload_val = 0.0,

    load_profile = :constant,
    Tload_step1_val = 40.0,
    Tload_step2_val = 80.0,
    t_load_step1_val = 6.0,
    t_load_step2_val = 8.0,

    # Estimator / observer parameters
    include_observer = false,
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

        Vll_nom = Vll_nom_val
        f_nom = f_nom_val
        f_ref = f_ref_val

        Tload = Tload_val

        # Observer parameters.
        # Lss and Lrr match the names used in the original MATLAB observer.
        Lss = Lls_val + Lm_val
        Lrr = Llr_val + Lm_val
        tau_r = tau_r_val
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

        Vll_nom = Vll_nom,
        f_nom = f_nom,
        f_ref = f_ref,

        Tload = Tload,

        Lss = Lss,
        Lrr = Lrr,
        tau_r = tau_r,
    )

    # ============================================================
    # Variables
    # ============================================================

    @variables begin
        # --------------------------------------------------------
        # Profile / reference generator variables
        # --------------------------------------------------------
        f_cmd(t)
        Tload_cmd(t)

        # --------------------------------------------------------
        # Scalar V/f controller variables
        # --------------------------------------------------------
        θs(t) = 0.0
        Vphase_peak(t)

        va(t)
        vb(t)
        vc(t)

        vsα(t)
        vsβ(t)

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
    end

    vars = (
        f_cmd = f_cmd,
        Tload_cmd = Tload_cmd,

        θs = θs,
        Vphase_peak = Vphase_peak,

        va = va,
        vb = vb,
        vc = vc,

        vsα = vsα,
        vsβ = vsβ,

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
    )

    # ============================================================
    # Build subsystem equations
    # ============================================================

    freq_eqs = build_frequency_command_eqs(
        f_cmd;
        frequency_profile = frequency_profile,
        f_ref = f_ref,
        f_ref_val = f_ref_val,
        f_step_val = f_step_val,
        t_hold_val = t_hold_val,
        t_step_val = t_step_val,
    )

    load_eqs = build_load_torque_eqs(
        Tload_cmd;
        load_profile = load_profile,
        Tload = Tload,
        Tload_step1_val = Tload_step1_val,
        Tload_step2_val = Tload_step2_val,
        t_load_step1_val = t_load_step1_val,
        t_load_step2_val = t_load_step2_val,
    )

    control_eqs = build_scalar_vf_control_eqs(vars, pars)

    plant_eqs = build_induction_machine_alpha_beta_eqs(vars, pars)

    observer_eqs = Equation[]

    if include_observer
        observer_eqs = build_rotor_flux_observer_eqs(vars, pars)
    end

    # ============================================================
    # Full system
    # ============================================================

    eqs = vcat(
        freq_eqs,
        load_eqs,
        control_eqs,
        plant_eqs,
        observer_eqs,
    )

    @named sys = ODESystem(eqs, t)
    return structural_simplify(sys)
end


# The `build_scalar_im_model` alias lives in the main module now. An extension
# cannot introduce a new binding into its parent's namespace, and the alias is
# exported from there — see src/IM_AWES_bench.jl.


"""
    simulate_scalar_im(; kwargs...)

Builds and simulates the scalar-controlled induction machine.

This is the standard function called from scripts.
"""
function simulate_scalar_im(;
    # ------------------------------------------------------------
    # Simulation time
    # ------------------------------------------------------------
    tspan = (0.0, 2.0),

    # ------------------------------------------------------------
    # Frequency profile
    # ------------------------------------------------------------
    frequency_profile = :constant,
    f_ref_val = 25.0,
    f_step_val = 5.0,
    t_hold_val = 1.0,
    t_step_val = 1.0,

    # ------------------------------------------------------------
    # Load torque profile
    # ------------------------------------------------------------
    load_profile = :constant,
    Tload_step1_val = 40.0,
    Tload_step2_val = 80.0,
    t_load_step1_val = 6.0,
    t_load_step2_val = 8.0,

    # ------------------------------------------------------------
    # Observer
    # ------------------------------------------------------------
    include_observer = false,
    tau_r_val = 0.3869971696,

    # ------------------------------------------------------------
    # Solver options
    # ------------------------------------------------------------
    solver = Rodas5P(),
    reltol = 1e-6,
    abstol = 1e-8,

    # ------------------------------------------------------------
    # Other model parameters
    # ------------------------------------------------------------
    kwargs...
)

    sys = build_scalar_im_system(;
        frequency_profile = frequency_profile,
        f_ref_val = f_ref_val,
        f_step_val = f_step_val,
        t_hold_val = t_hold_val,
        t_step_val = t_step_val,

        load_profile = load_profile,
        Tload_step1_val = Tload_step1_val,
        Tload_step2_val = Tload_step2_val,
        t_load_step1_val = t_load_step1_val,
        t_load_step2_val = t_load_step2_val,

        include_observer = include_observer,
        tau_r_val = tau_r_val,

        kwargs...
    )

    prob = ODEProblem(sys, [], tspan)

    # ============================================================
    # Force solver stops at known discontinuities
    # ============================================================

    tstops = Float64[]

    append!(
        tstops,
        frequency_profile_tstops(
            frequency_profile = frequency_profile,
            f_ref_val = f_ref_val,
            f_step_val = f_step_val,
            t_hold_val = t_hold_val,
            t_step_val = t_step_val,
            tspan = tspan,
        ),
    )

    append!(
        tstops,
        load_profile_tstops(
            load_profile = load_profile,
            t_load_step1_val = t_load_step1_val,
            t_load_step2_val = t_load_step2_val,
            tspan = tspan,
        ),
    )

    tstops = unique(sort(tstops))

    # ============================================================
    # Solve
    # ============================================================

    if isempty(tstops)
        sol = solve(
            prob,
            solver;
            reltol = reltol,
            abstol = abstol,
        )
    else
        sol = solve(
            prob,
            solver;
            reltol = reltol,
            abstol = abstol,
            tstops = tstops,
        )
    end

    return sol, sys
end