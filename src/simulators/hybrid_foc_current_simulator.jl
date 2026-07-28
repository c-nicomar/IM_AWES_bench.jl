# ============================================================
# Hybrid FOC current-loop simulator
# ============================================================
#
# Continuous plant:
#   induction machine in alpha-beta frame
#
# Discrete parts:
#   rotor flux observer
#   FOC current controller
#
# Integration:
#   fixed-step RK4 plant integration with controller held constant
#   during each control period.
# ============================================================

Base.@kwdef struct IMPlantParams
    Rs::Float64 = 0.21946
    Rr::Float64 = 0.11659
    Lls::Float64 = 4.28e-3
    Llr::Float64 = 4.28e-3
    Lm::Float64 = 40.84e-3
    p::Float64 = 2.0
    J::Float64 = 0.2685 + 0.1
    B::Float64 = 0.01298
end

Base.@kwdef struct IMPlantState
    isα::Float64 = 0.0
    isβ::Float64 = 0.0

    irα::Float64 = 0.0
    irβ::Float64 = 0.0

    ωm::Float64 = 0.0
    θm::Float64 = 0.0
end

Base.@kwdef struct IMPlantDerivatives
    disα::Float64 = 0.0
    disβ::Float64 = 0.0

    dirα::Float64 = 0.0
    dirβ::Float64 = 0.0

    dωm::Float64 = 0.0
    dθm::Float64 = 0.0
end

function im_fluxes(x::IMPlantState, p::IMPlantParams)
    Ls = p.Lls + p.Lm
    Lr = p.Llr + p.Lm

    ψsα = Ls * x.isα + p.Lm * x.irα
    ψsβ = Ls * x.isβ + p.Lm * x.irβ

    ψrα = p.Lm * x.isα + Lr * x.irα
    ψrβ = p.Lm * x.isβ + Lr * x.irβ

    return ψsα, ψsβ, ψrα, ψrβ
end

function im_torque(x::IMPlantState, p::IMPlantParams)
    ψsα, ψsβ, _, _ = im_fluxes(x, p)
    return 1.5 * p.p * (ψsα * x.isβ - ψsβ * x.isα)
end

function im_derivatives(
    x::IMPlantState,
    p::IMPlantParams;
    vsα::Float64,
    vsβ::Float64,
    Tload::Float64,
)

    Ls = p.Lls + p.Lm
    Lr = p.Llr + p.Lm

    detL = Ls * Lr - p.Lm^2

    _, _, ψrα, ψrβ = im_fluxes(x, p)

    # Flux derivative equations:
    #
    # dψsα = vsα - Rs isα
    # dψsβ = vsβ - Rs isβ
    #
    # rotor equations:
    # 0 = Rr irα + dψrα + pωm ψrβ
    # 0 = Rr irβ + dψrβ - pωm ψrα
    #
    # therefore:
    # dψrα = -Rr irα - pωm ψrβ
    # dψrβ = -Rr irβ + pωm ψrα

    dψsα = vsα - p.Rs * x.isα
    dψsβ = vsβ - p.Rs * x.isβ

    dψrα = -p.Rr * x.irα - p.p * x.ωm * ψrβ
    dψrβ = -p.Rr * x.irβ + p.p * x.ωm * ψrα

    # Convert flux derivatives to current derivatives:
    #
    # [dψs] = [Ls Lm] [dis]
    # [dψr]   [Lm Lr] [dir]

    disα = ( Lr * dψsα - p.Lm * dψrα) / detL
    dirα = (-p.Lm * dψsα + Ls * dψrα) / detL

    disβ = ( Lr * dψsβ - p.Lm * dψrβ) / detL
    dirβ = (-p.Lm * dψsβ + Ls * dψrβ) / detL

    Te = im_torque(x, p)

    dωm = (Te + Tload - p.B * x.ωm) / p.J
    dθm = x.ωm

    return IMPlantDerivatives(
        disα = disα,
        disβ = disβ,
        dirα = dirα,
        dirβ = dirβ,
        dωm = dωm,
        dθm = dθm,
    )
end

function add_scaled(x::IMPlantState, dx::IMPlantDerivatives, h::Float64)
    return IMPlantState(
        isα = x.isα + h * dx.disα,
        isβ = x.isβ + h * dx.disβ,

        irα = x.irα + h * dx.dirα,
        irβ = x.irβ + h * dx.dirβ,

        ωm = x.ωm + h * dx.dωm,
        θm = x.θm + h * dx.dθm,
    )
end

function rk4_step(
    x::IMPlantState,
    p::IMPlantParams,
    h::Float64;
    vsα::Float64,
    vsβ::Float64,
    Tload::Float64,
)

    k1 = im_derivatives(x, p; vsα = vsα, vsβ = vsβ, Tload = Tload)
    k2 = im_derivatives(add_scaled(x, k1, h / 2), p; vsα = vsα, vsβ = vsβ, Tload = Tload)
    k3 = im_derivatives(add_scaled(x, k2, h / 2), p; vsα = vsα, vsβ = vsβ, Tload = Tload)
    k4 = im_derivatives(add_scaled(x, k3, h), p; vsα = vsα, vsβ = vsβ, Tload = Tload)

    return IMPlantState(
        isα = x.isα + h / 6 * (k1.disα + 2k2.disα + 2k3.disα + k4.disα),
        isβ = x.isβ + h / 6 * (k1.disβ + 2k2.disβ + 2k3.disβ + k4.disβ),

        irα = x.irα + h / 6 * (k1.dirα + 2k2.dirα + 2k3.dirα + k4.dirα),
        irβ = x.irβ + h / 6 * (k1.dirβ + 2k2.dirβ + 2k3.dirβ + k4.dirβ),

        ωm = x.ωm + h / 6 * (k1.dωm + 2k2.dωm + 2k3.dωm + k4.dωm),
        θm = x.θm + h / 6 * (k1.dθm + 2k2.dθm + 2k3.dθm + k4.dθm),
    )
end

function current_reference_steps(
    t;
    t_id_step = 1.0,
    t_iq_pos_step = 3.0,
    t_iq_neg_step = 6.0,
    t_iq_zero_step = 9.0,
    isd_ref_mag = 5.0,
    isq_ref_pos = 5.0,
    isq_ref_neg = -5.0,
)

    isd_ref = t < t_id_step ? 0.0 : isd_ref_mag

    if t < t_iq_pos_step
        isq_ref = 0.0
    elseif t < t_iq_neg_step
        isq_ref = isq_ref_pos
    elseif t < t_iq_zero_step
        isq_ref = isq_ref_neg
    else
        isq_ref = 0.0
    end

    return isd_ref, isq_ref
end

function load_torque_profile(
    t;
    load_profile::Symbol = :constant,
    Tload = 0.0,
    Tload_step1 = 40.0,
    Tload_step2 = 80.0,
    t_load_step1 = 6.0,
    t_load_step2 = 8.0,
)

    if load_profile == :constant
        return Tload
    elseif load_profile == :steps
        if t < t_load_step1
            return 0.0
        elseif t < t_load_step2
            return Tload_step1
        else
            return Tload_step2
        end
    else
        error("Unknown load_profile = $load_profile.")
    end
end

function inverse_park_voltage(vsd, vsq, theta_e)
    vsα = cos(theta_e) * vsd - sin(theta_e) * vsq
    vsβ = sin(theta_e) * vsd + cos(theta_e) * vsq
    return vsα, vsβ
end

function simulate_foc_current_hybrid(;
    t_end = 12.0,
    Ts = 100e-6,
    plant_substeps = 1,

    # Current reference steps
    t_id_step = 1.0,
    t_iq_pos_step = 3.0,
    t_iq_neg_step = 6.0,
    t_iq_zero_step = 9.0,

    isd_ref_mag = 5.0,
    isq_ref_pos = 5.0,
    isq_ref_neg = -5.0,

    # Load
    load_profile::Symbol = :constant,
    Tload = 0.0,
    Tload_step1 = 40.0,
    Tload_step2 = 80.0,
    t_load_step1 = 6.0,
    t_load_step2 = 8.0,

    # Machine
    Rs = 0.21946,
    Rr = 0.11659,
    Lls = 4.28e-3,
    Llr = 4.28e-3,
    Lm = 40.84e-3,
    p_pairs = 2.0,
    J = 0.2685 + 0.1,
    B = 0.01298,

    # Controller
    Vs_max = 310.0,
    Is_max = 40.0,
    use_filter = true,
    use_feedforward = true,
    use_saturation = true,
    use_antiwindup = true,
)

    plant_p = IMPlantParams(
        Rs = Rs,
        Rr = Rr,
        Lls = Lls,
        Llr = Llr,
        Lm = Lm,
        p = p_pairs,
        J = J,
        B = B,
    )

    Lss = Lls + Lm
    Lrr = Llr + Lm
    sigma_Lss = Lss - Lm^2 / Lrr
    k = Lm / Lrr
    tau_r = Lrr / Rr

    obs_p = RotorFluxObserverDiscreteParams(
        Lm = Lm,
        Lss = Lss,
        Lrr = Lrr,
        tau_r = tau_r,
        p = p_pairs,
        Ts = Ts,
    )

    ctrl_p = CurrentControllerDiscreteParams(
        Rs = Rs,
        sigma_Lss = sigma_Lss,
        k = k,
        Ts = Ts,
        Vs_max = Vs_max,
        Is_max = Is_max,
        use_filter = use_filter,
        use_feedforward = use_feedforward,
        use_saturation = use_saturation,
        use_antiwindup = use_antiwindup,
    )

    x = IMPlantState()
    obs_state = RotorFluxObserverDiscreteState()
    ctrl_state = CurrentControllerDiscreteState()

    N = Int(floor(t_end / Ts)) + 1

    # Preallocate result vectors
    time = Vector{Float64}(undef, N)

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

    sat_vec = Vector{Float64}(undef, N)
    vs_mod_unsat_vec = Vector{Float64}(undef, N)

    h = Ts / plant_substeps

    for kstep in 1:N
        t = (kstep - 1) * Ts

        # --------------------------------------------------------
        # References and load
        # --------------------------------------------------------

        isd_ref, isq_ref = current_reference_steps(
            t;
            t_id_step = t_id_step,
            t_iq_pos_step = t_iq_pos_step,
            t_iq_neg_step = t_iq_neg_step,
            t_iq_zero_step = t_iq_zero_step,
            isd_ref_mag = isd_ref_mag,
            isq_ref_pos = isq_ref_pos,
            isq_ref_neg = isq_ref_neg,
        )

        Tload = load_torque_profile(
            t;
            load_profile = load_profile,
            Tload = Tload,
            Tload_step1 = Tload_step1,
            Tload_step2 = Tload_step2,
            t_load_step1 = t_load_step1,
            t_load_step2 = t_load_step2,
        )

        # --------------------------------------------------------
        # Observer update using measured plant currents and angle
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
        # Current controller update
        # --------------------------------------------------------

        ctrl = current_controller_step!(
            ctrl_state,
            ctrl_p;
            isd_ref = isd_ref,
            isq_ref = isq_ref,
            isd_med = obs.i_sd_e,
            isq_med = obs.i_sq_e,
            omega_e = obs.omega_e,
            lambda_rd = obs.lambda_rd_e,
            reset = false,
        )

        # --------------------------------------------------------
        # Voltage command dq -> alpha-beta, held during Ts
        # --------------------------------------------------------

        vsα_hold, vsβ_hold = inverse_park_voltage(ctrl.vsd, ctrl.vsq, obs.theta_e)

        # --------------------------------------------------------
        # Store values before plant update
        # --------------------------------------------------------

        time[kstep] = t

        isd_ref_vec[kstep] = isd_ref
        isq_ref_vec[kstep] = isq_ref

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

        Tload_vec[kstep] = Tload

        isα_vec[kstep] = x.isα
        isβ_vec[kstep] = x.isβ

        sat_vec[kstep] = ctrl.saturado ? 1.0 : 0.0
        vs_mod_unsat_vec[kstep] = ctrl.vs_mod_unsat

        # --------------------------------------------------------
        # Continuous plant integration over one controller sample
        # --------------------------------------------------------

        for _ in 1:plant_substeps
            x = rk4_step(
                x,
                plant_p,
                h;
                vsα = vsα_hold,
                vsβ = vsβ_hold,
                Tload = Tload,
            )
        end
    end

    return (
        t = time,

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

        saturation = sat_vec,
        vs_mod_unsat = vs_mod_unsat_vec,
    )
end