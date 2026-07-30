# ============================================================
# Hybrid FOC speed-control simulator:
# constrained-MTPA outer speed loop + inner current loop
# ============================================================
#
# This simulator intentionally mirrors hybrid_foc_speed_f1_simulator.jl
# so F1 and MTPA can be compared with the same:
#
#   - induction-machine plant
#   - rotor-flux observer
#   - discrete inner current controller
#   - load-torque estimator
#   - speed and load profiles
#
# The shared helpers speed_reference_ramp_profile and
# interp_profile_linear are defined by hybrid_foc_speed_f1_simulator.jl,
# which must be included before this file in InductionMachineDrives.jl.
#
# Mechanical sign convention:
#
#   J*dω/dt = Te + TL - B*ω
#
# Positive TL therefore pulls toward positive speed.
# ============================================================


"""
    simulate_foc_speed_mtpa_hybrid(; kwargs...)

Run the hybrid FOC speed-control simulation with a **constrained-MTPA** outer
loop and return the logged signals as a `NamedTuple` of equal-length vectors,
one entry per control sample.

Deliberately the twin of [`simulate_foc_speed_f1_hybrid`](@ref): same plant,
same rotor-flux observer, same inner current controller, same load estimator,
same speed and load profiles. The single difference is the flux policy — F1
holds `isd` at `isd_nom` regardless of load, while
[`outer_speed_flux_mtpa_step!`](@ref) picks `isd` per sample to minimise stator
current for the torque being demanded, subject to a flux floor and a torque
reserve. Running both with identical keywords is how the efficiency benefit is
measured.

# Keyword arguments

Every keyword of [`simulate_foc_speed_f1_hybrid`](@ref) is accepted with the
same meaning and the same defaults, except that the F1 field-weakening settings
have no counterpart here. In brief: `t_end`, `Ts`, `plant_substeps`; the
internal ramp profile `t_ramp_up_start`, `t_ramp_up_end`, `t_hold_end`,
`t_ramp_down_end`, `wm_ref_high_rpm`; profile playback via
`speed_reference_source`, `load_source`, `profile_time`,
`profile_speed_ref_rpm`, `profile_load_torque_Nm`, `profile_torque_sign`; the
internal load profile `load_profile`, `Tload`, `Tload_step1`, `Tload_step2`,
`t_load_step1`, `t_load_step2`; load estimation `load_estimator`,
`use_load_feedforward`, `load_ff_sign = -1.0`, `TL_kalman_R`,
`TL_kalman_q_omega`, `TL_kalman_q_TL`, `TL_kalman_limit_positive = false`; the
machine parameters `Rs`, `Rr`, `Lls`, `Llr`, `Lm`, `p_pairs`, `J`, `B`; the
outer-loop limits `isd_nom`, `outer_Is_max`, `isd_min`, `Te_max`, `wm_dot_max`,
`id_dot_max`; the inner controller `Vs_max`, `Is_max`, `use_filter`,
`use_feedforward`, `use_saturation`, `use_antiwindup`; and the `plant_*_scale`,
`obs_*_scale`, `ctrl_*_scale` mismatch multipliers.

The keywords specific to this simulator are:

- `lambda_rd_floor = 0.35`: minimum rotor flux in Wb held regardless of torque
  demand. Without it, pure MTPA would let the flux collapse near zero torque and
  the machine would be slow to respond. Set to `0` to disable.
- `Te_reserve = 45.0`: torque in N·m that must stay achievable within `Is_max`
  at all times, converted internally into a second flux floor. This is the knob
  that trades efficiency for transient headroom.
- `tau_f_wm = 10e-3`, `ts_wm = 500e-3`, `ts_dist_wm = 3.0`: speed filter time
  constant and PI design times in s. Unlike the F1 simulator, these are exposed
  here; the defaults match the MATLAB F3 controller.

Mechanical sign convention is unchanged: `J*dω/dt = Te + TL - B*ω`, so a
positive `TL` pulls toward positive speed and `load_ff_sign` stays `-1.0`.

# Returned signals

All the fields returned by [`simulate_foc_speed_f1_hybrid`](@ref) except the
F1-specific field-weakening diagnostics, plus the MTPA diagnostics that show
which constraint set the flux at each instant: `lambda_ref_out` (Wb), `Kt_isd`
(N·m/A²), `isd_mtpa`, `isd_floor`, `isd_reserve`, `isd_desired`, `isq_max_disp`
(all A), `Te_current_limited` (N·m) and `torque_current_limited` (flag).
Comparing `isd_desired` against `isd_mtpa`, `isd_floor` and `isd_reserve` tells
you directly whether the run is genuinely operating at the MTPA optimum or is
being held off it by one of the constraints.

!!! note "Include order"
    This simulator reuses `interp_profile_linear` and the other profile-playback
    helpers defined in `src/simulators/hybrid_foc_speed_f1_simulator.jl`, which
    must therefore be included before this file.
"""
function simulate_foc_speed_mtpa_hybrid(;
    t_end = 12.0,
    Ts = 100e-6,
    plant_substeps = 1,

    # ------------------------------------------------------------
    # Internal speed reference profile
    # ------------------------------------------------------------
    t_ramp_up_start = 1.0,
    t_ramp_up_end = 4.0,
    t_hold_end = 7.0,
    t_ramp_down_end = 10.0,
    wm_ref_high_rpm = 500.0,

    # ------------------------------------------------------------
    # External profile support
    # ------------------------------------------------------------
    speed_reference_source::Symbol = :ramp,   # :ramp or :profile
    load_source::Symbol = :internal,          # :internal or :profile

    profile_time = Float64[],
    profile_speed_ref_rpm = Float64[],
    profile_load_torque_Nm = Float64[],

    # If CSV torque already follows:
    #   positive TL pulls toward positive speed
    # use +1.
    # If CSV has opposite sign, use -1.
    profile_torque_sign = 1.0,

    # ------------------------------------------------------------
    # Internal load profile
    # ------------------------------------------------------------
    load_profile::Symbol = :steps,
    Tload = 0.0,
    Tload_step1 = 20.0,
    Tload_step2 = 0.0,
    t_load_step1 = 5.0,
    t_load_step2 = 8.0,

    # ------------------------------------------------------------
    # Load estimator / feedforward
    # ------------------------------------------------------------
    # Options:
    #   :none
    #   :actual
    #   :kalman
    load_estimator::Symbol = :none,
    use_load_feedforward = false,

    # With convention J*dω = Te + TL - Bω:
    #   Te_ff = J*dωref + B*ωref - TL_est
    # therefore default load feedforward sign should be -1.
    load_ff_sign = -1.0,

    # Kalman estimator tuning
    TL_kalman_R = 0.01,
    TL_kalman_q_omega = 0.1,
    TL_kalman_q_TL = 6.0,
    TL_kalman_limit_positive = false,

    # ------------------------------------------------------------
    # Machine nominal parameters
    # ------------------------------------------------------------
    Rs = 0.21946,
    Rr = 0.11659,
    Lls = 4.28e-3,
    Llr = 4.28e-3,
    Lm = 40.84e-3,
    p_pairs = 2.0,
    J = 0.2685 + 0.1,
    B = 0.01298,

    # ------------------------------------------------------------
    # Outer-loop parameters
    # ------------------------------------------------------------
    isd_nom = 23.04579328,
    outer_Is_max = 40.0,
    isd_min = 5.0,
    Te_max = 124.0419647,

    # Speed reference ramp limit inside outer loop.
    # For exact profile playback, pass wm_dot_max = 1e6 or similarly high.
    wm_dot_max = 100.0,

    id_dot_max = 600.0,

    # ------------------------------------------------------------
    # Constrained-MTPA parameters
    # ------------------------------------------------------------
    lambda_rd_floor = 0.35,
    Te_reserve = 45.0,

    # Speed-loop tuning. Defaults match the MATLAB F3 controller.
    tau_f_wm = 10e-3,
    ts_wm = 500e-3,
    ts_dist_wm = 3.0,

    # ------------------------------------------------------------
    # Inner current-controller limits
    # ------------------------------------------------------------
    Vs_max = 310.0,
    Is_max = 40.0,

    # ------------------------------------------------------------
    # Current-controller options
    # ------------------------------------------------------------
    use_filter = true,
    use_feedforward = true,
    use_saturation = true,
    use_antiwindup = true,

    # ------------------------------------------------------------
    # Optional plant / observer / controller mismatch multipliers
    # ------------------------------------------------------------
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
    # Profile validation
    # ============================================================

    if speed_reference_source == :profile || load_source == :profile
        if isempty(profile_time)
            error("profile_time is empty, but profile mode was requested.")
        end

        if any(diff(profile_time) .<= 0.0)
            error("profile_time must be strictly increasing.")
        end

        if speed_reference_source == :profile && length(profile_speed_ref_rpm) != length(profile_time)
            error("profile_speed_ref_rpm must have the same length as profile_time.")
        end

        if load_source == :profile && length(profile_load_torque_Nm) != length(profile_time)
            error("profile_load_torque_Nm must have the same length as profile_time.")
        end
    end

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
    # Load torque Kalman estimator assumed parameters
    # ============================================================

    load_kalman_p = LoadTorqueKalmanParams(
        J = J_nom,
        B = B_nom,
        Ts = Ts,
        R = TL_kalman_R,
        q_omega = TL_kalman_q_omega,
        q_TL = TL_kalman_q_TL,
        limit_positive = TL_kalman_limit_positive,
    )

    # ============================================================
    # Outer-loop assumed parameters
    # ============================================================

    outer_p = OuterSpeedFluxMTPAParams(
        Ts = Ts,
        p = p_pairs,
        Lm = Lm_nom,
        Lrr = Lrr_nom,
        J = J_nom,
        B = B_nom,
        isd_nom = isd_nom,
        Is_max = outer_Is_max,
        isd_min = isd_min,
        Te_max = Te_max,
        wm_dot_max = wm_dot_max,
        id_dot_max = id_dot_max,
        lambda_rd_floor = lambda_rd_floor,
        Te_reserve = Te_reserve,
        tau_f_wm = tau_f_wm,
        ts_wm = ts_wm,
        ts_dist_wm = ts_dist_wm,
        use_load_feedforward = use_load_feedforward,
        load_ff_sign = load_ff_sign,
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
    load_kalman_state = LoadTorqueKalmanState()
    outer_state = OuterSpeedFluxMTPAState(id_ref_ant = isd_nom)
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

    lambda_ref_out_vec = Vector{Float64}(undef, N)
    Kt_isd_vec = Vector{Float64}(undef, N)
    isd_mtpa_vec = Vector{Float64}(undef, N)
    isd_floor_vec = Vector{Float64}(undef, N)
    isd_reserve_vec = Vector{Float64}(undef, N)
    isd_desired_vec = Vector{Float64}(undef, N)
    isq_max_disp_vec = Vector{Float64}(undef, N)
    Te_current_limited_vec = Vector{Float64}(undef, N)
    torque_current_limited_vec = Vector{Float64}(undef, N)

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
    TL_est_vec = Vector{Float64}(undef, N)
    omega_hat_load_vec = Vector{Float64}(undef, N)

    isα_vec = Vector{Float64}(undef, N)
    isβ_vec = Vector{Float64}(undef, N)

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

    vs_mod_unsat_vec = Vector{Float64}(undef, N)

    # ============================================================
    # Main hybrid simulation loop
    # ============================================================

    for kstep in 1:N
        t = (kstep - 1) * Ts

        # --------------------------------------------------------
        # Speed reference
        # --------------------------------------------------------

        if speed_reference_source == :ramp

            wm_ref = speed_reference_ramp_profile(
                t;
                t_ramp_up_start = t_ramp_up_start,
                t_ramp_up_end = t_ramp_up_end,
                t_hold_end = t_hold_end,
                t_ramp_down_end = t_ramp_down_end,
                wm_ref_high_rpm = wm_ref_high_rpm,
            )

        elseif speed_reference_source == :profile

            speed_ref_rpm_k = interp_profile_linear(
                t,
                profile_time,
                profile_speed_ref_rpm,
            )

            wm_ref = speed_ref_rpm_k * 2π / 60.0

        else

            error("Unknown speed_reference_source = $speed_reference_source. Use :ramp or :profile.")

        end

        # --------------------------------------------------------
        # Load torque
        # --------------------------------------------------------

        if load_source == :internal

            Tload_k = load_torque_profile(
                t;
                load_profile = load_profile,
                Tload = Tload,
                Tload_step1 = Tload_step1,
                Tload_step2 = Tload_step2,
                t_load_step1 = t_load_step1,
                t_load_step2 = t_load_step2,
            )

        elseif load_source == :profile

            Tload_k = profile_torque_sign * interp_profile_linear(
                t,
                profile_time,
                profile_load_torque_Nm,
            )

        else

            error("Unknown load_source = $load_source. Use :internal or :profile.")

        end

        # --------------------------------------------------------
        # Rotor-flux observer update
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
        # Load torque estimator
        # --------------------------------------------------------

        if load_estimator == :none

            TL_est = 0.0
            omega_hat_load = x.ωm

        elseif load_estimator == :actual

            TL_est = Tload_k
            omega_hat_load = x.ωm

        elseif load_estimator == :kalman

            load_obs = load_torque_kalman_step!(
                load_kalman_state,
                load_kalman_p;
                omega = x.ωm,
                Te = obs.Te_raw,
                reset = false,
            )

            TL_est = load_obs.TL_hat
            omega_hat_load = load_obs.omega_hat

        else

            error("Unknown load_estimator = $load_estimator. Use :none, :actual, or :kalman.")

        end

        # --------------------------------------------------------
        # Outer speed/flux loop
        # --------------------------------------------------------

        outer = outer_speed_flux_mtpa_step!(
            outer_state,
            outer_p;
            wm_ref = wm_ref,
            wm_med = x.ωm,
            TL_est = TL_est,
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
        # Reconstruct abc quantities from alpha-beta
        # --------------------------------------------------------
        #
        # Assumption:
        #   balanced three-phase system, zero sequence = 0

        ia_k = x.isα
        ib_k = -0.5 * x.isα + sqrt(3) / 2 * x.isβ
        ic_k = -0.5 * x.isα - sqrt(3) / 2 * x.isβ

        va_k = vsα_hold
        vb_k = -0.5 * vsα_hold + sqrt(3) / 2 * vsβ_hold
        vc_k = -0.5 * vsα_hold - sqrt(3) / 2 * vsβ_hold

        # Instantaneous stator electrical power.
        # Positive means electrical power entering the stator from the converter.
        Pelec_k = va_k * ia_k + vb_k * ib_k + vc_k * ic_k

        # Electromagnetic mechanical power.
        # Positive means electromagnetic torque delivering positive mechanical power.
        torque_k = im_torque(x, plant_p)
        Pmech_k = torque_k * x.ωm

        # External load mechanical power with the convention:
        #   J*dω = Te + TL - Bω
        #
        # Positive Pload means the external torque injects mechanical power
        # into the shaft in the positive-speed direction.
        Pload_k = Tload_k * x.ωm

        # Viscous friction loss.
        Pfric_k = plant_p.B * x.ωm^2

        # --------------------------------------------------------
        # Store values before plant update
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

        lambda_ref_out_vec[kstep] = outer.lambda_ref_out
        Kt_isd_vec[kstep] = outer.Kt_isd
        isd_mtpa_vec[kstep] = outer.isd_mtpa
        isd_floor_vec[kstep] = outer.isd_floor
        isd_reserve_vec[kstep] = outer.isd_reserve
        isd_desired_vec[kstep] = outer.isd_desired
        isq_max_disp_vec[kstep] = outer.isq_max_disp
        Te_current_limited_vec[kstep] = outer.Te_current_limited
        torque_current_limited_vec[kstep] = outer.torque_current_limited

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

        torque_vec[kstep] = torque_k
        torque_obs_vec[kstep] = obs.Te_raw

        flux_r_vec[kstep] = obs.flux_r_mod
        theta_e_vec[kstep] = obs.theta_e
        omega_e_vec[kstep] = obs.omega_e

        Tload_vec[kstep] = Tload_k
        TL_est_vec[kstep] = TL_est
        omega_hat_load_vec[kstep] = omega_hat_load

        isα_vec[kstep] = x.isα
        isβ_vec[kstep] = x.isβ

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
        sat_isd_vec[kstep] = outer.sat_isd
        sat_isq_vec[kstep] = outer.sat_isq
        sat_Te_vec[kstep] = outer.sat_Te

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

        Te_ref_out = Te_ref_out_vec,
        Te_PI = Te_PI_vec,
        Te_ff = Te_ff_vec,

        isd_ref = isd_ref_vec,
        isq_ref = isq_ref_vec,

        lambda_ref_out = lambda_ref_out_vec,
        Kt_isd = Kt_isd_vec,
        isd_mtpa = isd_mtpa_vec,
        isd_floor = isd_floor_vec,
        isd_reserve = isd_reserve_vec,
        isd_desired = isd_desired_vec,
        isq_max_disp = isq_max_disp_vec,
        Te_current_limited = Te_current_limited_vec,
        torque_current_limited = torque_current_limited_vec,

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
        TL_est = TL_est_vec,
        omega_hat_load = omega_hat_load_vec,

        isα = isα_vec,
        isβ = isβ_vec,

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

        vs_mod_unsat = vs_mod_unsat_vec,
    )
end