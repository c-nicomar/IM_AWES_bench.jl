using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using IM_AWES_bench
using MakieControlPlots
using CSV
using DataFrames

# ============================================================
# Simulation case definition
# ============================================================

case_name = "foc_speed_f1_awes_profile"

# FOC speed control using AWES offline test profiles:
#
# CSV inputs:
#   t_s
#   speed_ref_rpm
#   torque_ref_Nm
#
# Control:
#   outer loop: speed mode, T1, F1
#   inner loop: current controller
#   load torque estimator: Kalman
#
# Plant:
#   induction machine alpha-beta model

# ============================================================
# Load profile CSV
# ============================================================

profile_file = joinpath(@__DIR__, "..", "profiles", "delta_kite_13_ms_profiles.csv")

profile = CSV.read(profile_file, DataFrame)

profile_time = Float64.(profile.t_s)
profile_speed_ref_rpm = Float64.(profile.speed_ref_rpm)
profile_load_torque_Nm = Float64.(profile.torque_ref_Nm)

# Ensure profile starts at t = 0.0, even if the CSV did not.
profile_time = profile_time .- profile_time[1]

# Ensure profile is sorted by time.
idx = sortperm(profile_time)
profile_time = profile_time[idx]
profile_speed_ref_rpm = profile_speed_ref_rpm[idx]
profile_load_torque_Nm = profile_load_torque_Nm[idx]

# Remove duplicate time values if they exist.
keep = [true; diff(profile_time) .> 0.0]
profile_time = profile_time[keep]
profile_speed_ref_rpm = profile_speed_ref_rpm[keep]
profile_load_torque_Nm = profile_load_torque_Nm[keep]

t_end = profile_time[end]

println("Profile check:")
println("  file: ", profile_file)
println("  samples: ", length(profile_time))
println("  t start/end: ", profile_time[1], " / ", profile_time[end], " s")
println("  speed_ref rpm min/max: ", minimum(profile_speed_ref_rpm), " / ", maximum(profile_speed_ref_rpm))
println("  torque_ref Nm min/max: ", minimum(profile_load_torque_Nm), " / ", maximum(profile_load_torque_Nm))
println()

# ============================================================
# Simulation settings
# ============================================================

Ts = 100e-6
plant_substeps = 1

# ============================================================
# Profile playback settings
# ============================================================

speed_reference_source = :profile
load_source = :profile

# Mechanical sign convention:
#
#   TL + Te = J*dω/dt + B*ω
#
# equivalently:
#
#   J*dω/dt = Te + TL - B*ω
#
# Therefore:
#   positive Tload pulls toward positive speed.
#
# If torque_ref in the CSV already follows this convention, use +1.
# If it has the opposite sign, use -1.

profile_torque_sign = 1.0

# Disable internal speed-reference ramp limiting for profile playback.
# This makes wm_ref_ramp follow the CSV speed profile unless the profile
# has extremely sharp discontinuities.
wm_dot_max = 1e6

# ============================================================
# Load estimator / feedforward settings
# ============================================================
#
# load_estimator options:
#   :none
#   :actual
#   :kalman
#
# Recommended first tests:
#   load_estimator = :kalman, use_load_feedforward = false
#   load_estimator = :kalman, use_load_feedforward = true
#   load_estimator = :actual, use_load_feedforward = true

load_estimator = :kalman
use_load_feedforward = true

# With convention:
#
#   J*dω = Te + TL - Bω
#
# the ideal speed feedforward is:
#
#   Te_ff = J*dωref + B*ωref - TL_est
#
# so load_ff_sign must be -1.
load_ff_sign = -1.0

TL_kalman_R = 0.01
TL_kalman_q_omega = 0.1
TL_kalman_q_TL = 6.0

# In AWES profiles TL may be signed. Use false unless you are sure
# TL can only be positive.
TL_kalman_limit_positive = false

# ============================================================
# Controller limits
# ============================================================

Vs_max = 310.0
Is_max = 40.0

# F1 d-axis current reference
isd_nom = 23.04579328

# Current-controller options
use_filter = true
use_feedforward = true
use_saturation = true
use_antiwindup = true

# ============================================================
# Optional robustness / mismatch settings
# ============================================================

plant_Rr_scale = 1.0
obs_tau_r_scale = 1.0
ctrl_k_scale = 1.0

# ============================================================
# Run simulation
# ============================================================

res = simulate_foc_speed_f1_hybrid(
    t_end = t_end,
    Ts = Ts,
    plant_substeps = plant_substeps,

    speed_reference_source = speed_reference_source,
    load_source = load_source,

    profile_time = profile_time,
    profile_speed_ref_rpm = profile_speed_ref_rpm,
    profile_load_torque_Nm = profile_load_torque_Nm,
    profile_torque_sign = profile_torque_sign,

    wm_dot_max = wm_dot_max,

    load_estimator = load_estimator,
    use_load_feedforward = use_load_feedforward,
    load_ff_sign = load_ff_sign,

    TL_kalman_R = TL_kalman_R,
    TL_kalman_q_omega = TL_kalman_q_omega,
    TL_kalman_q_TL = TL_kalman_q_TL,
    TL_kalman_limit_positive = TL_kalman_limit_positive,

    Vs_max = Vs_max,
    Is_max = Is_max,
    outer_Is_max = Is_max,
    isd_nom = isd_nom,

    use_filter = use_filter,
    use_feedforward = use_feedforward,
    use_saturation = use_saturation,
    use_antiwindup = use_antiwindup,

    plant_Rr_scale = plant_Rr_scale,
    obs_tau_r_scale = obs_tau_r_scale,
    ctrl_k_scale = ctrl_k_scale,
)

# ============================================================
# Plot
# ============================================================

wm_ref_rpm = res.wm_ref .* 60 ./ (2π)
wm_ref_ramp_rpm = res.wm_ref_ramp .* 60 ./ (2π)
omega_hat_load_rpm = res.omega_hat_load .* 60 ./ (2π)

p_plot = plotx(
    res.t,

    # Speed
    [wm_ref_rpm, wm_ref_ramp_rpm, res.speed_rpm],

    # Torques
    [res.Te_ref_out, res.torque, res.torque_obs, res.Tload, res.TL_est],

    # dq currents
    [res.isd_ref, res.isd, res.isq_ref, res.isq],

    # abc currents
    [res.ia, res.ib, res.ic],

    # Power
    [res.Pelec ./ 1000, res.Pmech ./ 1000, -res.Pload ./ 1000, res.Pfric ./ 1000],

    # Rotor flux
    res.flux_r;

    xlabel = "Time [s]",
    ylabels = [
        "Speed [rpm]",
        "Torque [N m]",
        "dq current [A]",
        "abc current [A]",
        "Power [kW]",
        "Rotor flux [Wb]",
    ],
    labels = [
        ["Speed ref", "Speed ref ramp", "Measured speed"],
        ["Te ref", "Plant torque", "Observed torque", "AWES torque", "Estimated load torque"],
        ["id ref", "id obs", "iq ref", "iq obs"],
        ["ia", "ib", "ic"],
        ["Electrical stator power", "Mechanical EM power", "External load power", "Friction loss"],
        ["Rotor flux magnitude"],
    ],
    fig = "FOC speed control - AWES profile",
    title = "FOC speed control with AWES profile: currents, torque and power",
    yzoom = 1.20,
    legendsize = 8,
)

display(p_plot)

# ============================================================
# Save results
# ============================================================

results_dir = joinpath(@__DIR__, "..", "results")
mkpath(results_dir)

csv_file = joinpath(results_dir, case_name * ".csv")

df = DataFrame(
    t_s = collect(res.t),

    # Speed
    wm_ref_rad_s = collect(res.wm_ref),
    wm_ref_ramp_rad_s = collect(res.wm_ref_ramp),
    wm_meas_rad_s = collect(res.omega_m),
    omega_hat_load_rad_s = collect(res.omega_hat_load),

    wm_ref_rpm = collect(wm_ref_rpm),
    wm_ref_ramp_rpm = collect(wm_ref_ramp_rpm),
    speed_rpm = collect(res.speed_rpm),
    omega_hat_load_rpm = collect(omega_hat_load_rpm),

    e_wm_rad_s = collect(res.e_wm),

    # Torque
    Te_ref_out_Nm = collect(res.Te_ref_out),
    Te_PI_Nm = collect(res.Te_PI),
    Te_ff_Nm = collect(res.Te_ff),

    torque_Nm = collect(res.torque),
    torque_obs_Nm = collect(res.torque_obs),

    Tload_Nm = collect(res.Tload),
    TL_est_Nm = collect(res.TL_est),

    # dq currents
    isd_ref_A = collect(res.isd_ref),
    isq_ref_A = collect(res.isq_ref),

    isd_obs_A = collect(res.isd),
    isq_obs_A = collect(res.isq),

    isd_ref_lim_A = collect(res.isd_ref_lim),
    isq_ref_lim_A = collect(res.isq_ref_lim),

    # alpha-beta voltages and currents
    vsd_V = collect(res.vsd),
    vsq_V = collect(res.vsq),
    vs_alpha_V = collect(res.vsα),
    vs_beta_V = collect(res.vsβ),

    is_alpha_A = collect(res.isα),
    is_beta_A = collect(res.isβ),

    # abc reconstructed voltages and currents
    va_V = collect(res.va),
    vb_V = collect(res.vb),
    vc_V = collect(res.vc),

    ia_A = collect(res.ia),
    ib_A = collect(res.ib),
    ic_A = collect(res.ic),

    # Powers
    Pelec_W = collect(res.Pelec),
    Pmech_W = collect(res.Pmech),
    Pload_W = collect(res.Pload),
    Pfric_W = collect(res.Pfric),

    Pelec_kW = collect(res.Pelec ./ 1000),
    Pmech_kW = collect(res.Pmech ./ 1000),
    Pload_kW = collect(res.Pload ./ 1000),
    Pfric_kW = collect(res.Pfric ./ 1000),

    # Flux / angle
    flux_r_Wb = collect(res.flux_r),
    theta_e_rad = collect(res.theta_e),
    omega_e_rad_s = collect(res.omega_e),

    # Debug / saturation
    voltage_saturation = collect(res.saturation_current),
    sat_Te = collect(res.sat_Te),
    sat_isd = collect(res.sat_isd),
    sat_isq = collect(res.sat_isq),

    vs_mod_unsat_V = collect(res.vs_mod_unsat),
)

CSV.write(csv_file, df)

# ============================================================
# Console summary
# ============================================================

println("Saved simulation results to:")
println(csv_file)

println()
println("Simulation case: ", case_name)
println("Profile file: ", profile_file)
println("t_end = ", t_end, " s")
println("Ts = ", Ts, " s")
println("plant_substeps = ", plant_substeps)
println("load_estimator = ", load_estimator)
println("use_load_feedforward = ", use_load_feedforward)
println("profile_torque_sign = ", profile_torque_sign)
println("wm_dot_max = ", wm_dot_max, " rad/s^2")
println()
println("Final speed ref = ", wm_ref_rpm[end], " rpm")
println("Final speed ref ramp = ", wm_ref_ramp_rpm[end], " rpm")
println("Final speed = ", res.speed_rpm[end], " rpm")
println("Final speed error = ", res.e_wm[end], " rad/s")
println()
println("Final Te ref = ", res.Te_ref_out[end], " N m")
println("Final plant torque = ", res.torque[end], " N m")
println("Final observed torque = ", res.torque_obs[end], " N m")
println("Final AWES torque = ", res.Tload[end], " N m")
println("Final estimated load torque = ", res.TL_est[end], " N m")
println()
println("Final id ref = ", res.isd_ref[end], " A")
println("Final iq ref = ", res.isq_ref[end], " A")
println("Final id obs = ", res.isd[end], " A")
println("Final iq obs = ", res.isq[end], " A")
println("Final rotor flux = ", res.flux_r[end], " Wb")
println()
println("Final electrical stator power = ", res.Pelec[end] / 1000, " kW")
println("Final mechanical EM power = ", res.Pmech[end] / 1000, " kW")
println("Final external load power = ", res.Pload[end] / 1000, " kW")
println("Final friction loss = ", res.Pfric[end] / 1000, " kW")
println()
println("Press ENTER to close.")
readline()