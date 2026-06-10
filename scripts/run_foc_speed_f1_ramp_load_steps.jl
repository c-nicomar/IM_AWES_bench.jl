using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))

ENV["KMP_DUPLICATE_LIB_OK"] = "TRUE"

using IM_AWES_bench
using ControlPlots
using CSV
using DataFrames

# ============================================================
# Simulation case definition
# ============================================================

case_name = "foc_speed_f1_ramp_load_steps"

# Discrete FOC:
#   outer loop: speed mode, T1, F1
#   inner loop: current controller
#
# Speed reference:
#   0–1 s    0 rpm
#   1–4 s    ramp to 500 rpm
#   4–7 s    hold 500 rpm
#   7–10 s   ramp down to 0 rpm
#   10–12 s  hold 0 rpm
#
# Load:
#   0–5 s    0 Nm
#   5–8 s    20 Nm
#   8–12 s   0 Nm

# ------------------------------------------------------------
# Simulation settings
# ------------------------------------------------------------

t_end = 12.0
Ts = 100e-6
plant_substeps = 1

# ------------------------------------------------------------
# Speed reference profile
# ------------------------------------------------------------

t_ramp_up_start = 1.0
t_ramp_up_end = 4.0
t_hold_end = 7.0
t_ramp_down_end = 10.0
wm_ref_high_rpm = 500.0

# ------------------------------------------------------------
# Load torque disturbance
# ------------------------------------------------------------

load_profile = :steps
Tload = 0.0
Tload_step1 = 20.0
Tload_step2 = 0.0
t_load_step1 = 5.0
t_load_step2 = 8.0

# ------------------------------------------------------------
# Load estimator / feedforward
# ------------------------------------------------------------
#
# load_estimator options:
#   :none    -> TL_est = 0
#   :actual  -> ideal debugging, TL_est = actual load
#   :kalman  -> discrete Kalman load torque estimator
#
# For this script we do NOT use the estimator or feedforward.

load_estimator = :none
use_load_feedforward = false
load_ff_sign = -1.0

# ------------------------------------------------------------
# Controller limits
# ------------------------------------------------------------

Vs_max = 310.0
Is_max = 40.0

# F1 d-axis current reference
isd_nom = 23.04579328

# Current-controller options
use_filter = true
use_feedforward = true
use_saturation = true
use_antiwindup = true

# ------------------------------------------------------------
# Optional robustness / mismatch settings
# ------------------------------------------------------------

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

    t_ramp_up_start = t_ramp_up_start,
    t_ramp_up_end = t_ramp_up_end,
    t_hold_end = t_hold_end,
    t_ramp_down_end = t_ramp_down_end,
    wm_ref_high_rpm = wm_ref_high_rpm,

    load_profile = load_profile,
    Tload = Tload,
    Tload_step1 = Tload_step1,
    Tload_step2 = Tload_step2,
    t_load_step1 = t_load_step1,
    t_load_step2 = t_load_step2,

    load_estimator = load_estimator,
    use_load_feedforward = use_load_feedforward,
    load_ff_sign = load_ff_sign,

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

p_plot = plotx(
    res.t,
    [wm_ref_rpm, wm_ref_ramp_rpm, res.speed_rpm],
    [res.Te_ref_out, res.torque, res.torque_obs, res.Tload],
    [res.isd_ref, res.isd],
    [res.isq_ref, res.isq],
    res.flux_r,
    [res.vsd, res.vsq],
    [res.saturation_current, res.sat_Te, res.sat_isd, res.sat_isq];
    xlabel = "Time [s]",
    ylabels = [
        "Speed [rpm]",
        "Torque [N m]",
        "d current [A]",
        "q current [A]",
        "Rotor flux [Wb]",
        "Voltage dq [V]",
        "Saturation [-]",
    ],
    labels = [
        ["wm ref", "wm ref ramp", "wm measured"],
        ["Te ref", "Plant torque", "Observed torque", "Load torque"],
        ["id ref", "id obs"],
        ["iq ref", "iq obs"],
        ["Rotor flux magnitude"],
        ["vsd", "vsq"],
        ["Current voltage sat", "Te sat", "id sat", "iq sat"],
    ],
    fig = "FOC speed F1",
    title = "FOC outer speed loop: mode_control=2, mode_torque=1, mode_flux=1",
    yzoom = 1.20,
    legend_size = 8,
    loc = "best",
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

    wm_ref_rad_s = collect(res.wm_ref),
    wm_ref_ramp_rad_s = collect(res.wm_ref_ramp),
    wm_meas_rad_s = collect(res.omega_m),

    wm_ref_rpm = collect(wm_ref_rpm),
    wm_ref_ramp_rpm = collect(wm_ref_ramp_rpm),
    speed_rpm = collect(res.speed_rpm),

    e_wm_rad_s = collect(res.e_wm),

    Te_ref_out_Nm = collect(res.Te_ref_out),
    Te_PI_Nm = collect(res.Te_PI),
    Te_ff_Nm = collect(res.Te_ff),

    torque_Nm = collect(res.torque),
    torque_obs_Nm = collect(res.torque_obs),

    isd_ref_A = collect(res.isd_ref),
    isq_ref_A = collect(res.isq_ref),

    isd_obs_A = collect(res.isd),
    isq_obs_A = collect(res.isq),

    isd_ref_lim_A = collect(res.isd_ref_lim),
    isq_ref_lim_A = collect(res.isq_ref_lim),

    vsd_V = collect(res.vsd),
    vsq_V = collect(res.vsq),
    vs_alpha_V = collect(res.vsα),
    vs_beta_V = collect(res.vsβ),

    flux_r_Wb = collect(res.flux_r),
    theta_e_rad = collect(res.theta_e),
    omega_e_rad_s = collect(res.omega_e),

    Tload_Nm = collect(res.Tload),
    TL_est_Nm = collect(res.TL_est),

    is_alpha_A = collect(res.isα),
    is_beta_A = collect(res.isβ),

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
println("t_end = ", t_end, " s")
println("Ts = ", Ts, " s")
println("plant_substeps = ", plant_substeps)
println()
println("Final wm ref = ", wm_ref_rpm[end], " rpm")
println("Final speed = ", res.speed_rpm[end], " rpm")
println("Final speed error = ", res.e_wm[end], " rad/s")
println("Final Te ref = ", res.Te_ref_out[end], " N m")
println("Final plant torque = ", res.torque[end], " N m")
println("Final observed torque = ", res.torque_obs[end], " N m")
println("Final id ref = ", res.isd_ref[end], " A")
println("Final iq ref = ", res.isq_ref[end], " A")
println("Final id obs = ", res.isd[end], " A")
println("Final iq obs = ", res.isq[end], " A")
println("Final rotor flux = ", res.flux_r[end], " Wb")
println()
println("Press ENTER to close.")
readline()