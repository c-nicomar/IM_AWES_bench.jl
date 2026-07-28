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

case_name = "foc_torque_f1_steps"

# Discrete FOC:
#   outer loop: torque mode, T1, F1
#   inner loop: current controller
#
# Torque reference:
#   0–2 s       Te_ref = 0 Nm
#   2–6 s       Te_ref = +2 Nm
#   6–10 s      Te_ref = -2 Nm
#   10 s onward Te_ref = 0 Nm

# ------------------------------------------------------------
# Simulation settings
# ------------------------------------------------------------

t_end = 12.0
Ts = 100e-6
plant_substeps = 1

# ------------------------------------------------------------
# Torque reference profile
# ------------------------------------------------------------

t_Te_pos_step = 2.0
t_Te_neg_step = 6.0
t_Te_zero_step = 10.0

Te_ref_pos = 2.0
Te_ref_neg = -2.0

# ------------------------------------------------------------
# Load torque
# ------------------------------------------------------------

load_profile = :constant
Tload = 0.0

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

res = simulate_foc_torque_f1_hybrid(
    t_end = t_end,
    Ts = Ts,
    plant_substeps = plant_substeps,

    t_Te_pos_step = t_Te_pos_step,
    t_Te_neg_step = t_Te_neg_step,
    t_Te_zero_step = t_Te_zero_step,
    Te_ref_pos = Te_ref_pos,
    Te_ref_neg = Te_ref_neg,

    load_profile = load_profile,
    Tload = Tload,

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

p_plot = plotx(
    res.t,
    [res.Te_ref_ext, res.Te_ref_out, res.torque, res.torque_obs],
    [res.isd_ref, res.isd],
    [res.isq_ref, res.isq],
    res.speed_rpm,
    res.flux_r,
    [res.vsd, res.vsq],
    [res.saturation_current, res.sat_Te, res.sat_isd, res.sat_isq];
    xlabel = "Time [s]",
    ylabels = [
        "Torque [N m]",
        "d current [A]",
        "q current [A]",
        "Speed [rpm]",
        "Rotor flux [Wb]",
        "Voltage dq [V]",
        "Saturation [-]",
    ],
    labels = [
        ["Te ref ext", "Te ref out", "Plant torque", "Observed torque"],
        ["id ref", "id obs"],
        ["iq ref", "iq obs"],
        ["Rotor speed"],
        ["Rotor flux magnitude"],
        ["vsd", "vsq"],
        ["Current voltage sat", "Te sat", "id sat", "iq sat"],
    ],
    fig = "FOC torque F1",
    title = "FOC outer torque loop: mode_control=1, mode_torque=1, mode_flux=1",
    yzoom = 1.20,
    legendsize = 10,
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

    Te_ref_ext_Nm = collect(res.Te_ref_ext),
    Te_ref_out_Nm = collect(res.Te_ref_out),

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

    speed_rpm = collect(res.speed_rpm),
    omega_m_rad_s = collect(res.omega_m),

    flux_r_Wb = collect(res.flux_r),
    theta_e_rad = collect(res.theta_e),
    omega_e_rad_s = collect(res.omega_e),

    Tload_Nm = collect(res.Tload),

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
println("Final Te ref = ", res.Te_ref_out[end], " N m")
println("Final plant torque = ", res.torque[end], " N m")
println("Final observed torque = ", res.torque_obs[end], " N m")
println("Final id ref = ", res.isd_ref[end], " A")
println("Final iq ref = ", res.isq_ref[end], " A")
println("Final id obs = ", res.isd[end], " A")
println("Final iq obs = ", res.isq[end], " A")
println("Final speed = ", res.speed_rpm[end], " rpm")
println("Final rotor flux = ", res.flux_r[end], " Wb")
