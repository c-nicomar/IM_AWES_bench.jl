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

case_name = "foc_current_hybrid_steps"

# Discrete FOC current loop + continuous induction machine plant.
#
# References:
#   0–1 s      id_ref = 0 A, iq_ref = 0 A
#   1–3 s      id_ref = 5 A, iq_ref = 0 A
#   3–6 s      id_ref = 5 A, iq_ref = 5 A
#   6–9 s      id_ref = 5 A, iq_ref = -5 A
#   9 s onward id_ref = 5 A, iq_ref = 0 A

# ------------------------------------------------------------
# Simulation settings
# ------------------------------------------------------------

t_end = 12.0
Ts = 100e-6

# Start with 1 substep. If unstable, try 2, 5, or 10.
plant_substeps = 1

# ------------------------------------------------------------
# Current reference profile
# ------------------------------------------------------------

t_id_step = 1.0
t_iq_pos_step = 3.0
t_iq_neg_step = 6.0
t_iq_zero_step = 9.0

isd_ref_mag = 5.0
isq_ref_pos = 5.0
isq_ref_neg = -5.0

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

# For first validation, start simple. Once working, set these true.
use_filter = true
use_feedforward = true
use_saturation = true
use_antiwindup = true

# ============================================================
# Run simulation
# ============================================================

res = simulate_foc_current_hybrid(
    t_end = t_end,
    Ts = Ts,
    plant_substeps = plant_substeps,

    t_id_step = t_id_step,
    t_iq_pos_step = t_iq_pos_step,
    t_iq_neg_step = t_iq_neg_step,
    t_iq_zero_step = t_iq_zero_step,

    isd_ref_mag = isd_ref_mag,
    isq_ref_pos = isq_ref_pos,
    isq_ref_neg = isq_ref_neg,

    load_profile = load_profile,
    Tload = Tload,

    Vs_max = Vs_max,
    Is_max = Is_max,

    use_filter = use_filter,
    use_feedforward = use_feedforward,
    use_saturation = use_saturation,
    use_antiwindup = use_antiwindup,
)

# ============================================================
# Plot
# ============================================================

p_plot = plotx(
    res.t,
    [res.isd_ref, res.isd],
    [res.isq_ref, res.isq],
    [res.torque, res.torque_obs],
    res.speed_rpm,
    res.flux_r,
    [res.vsd, res.vsq],
    res.saturation;
    xlabel = "Time [s]",
    ylabels = [
        "d current [A]",
        "q current [A]",
        "Torque [N m]",
        "Speed [rpm]",
        "Rotor flux [Wb]",
        "Voltage dq [V]",
        "Saturation [-]",
    ],
    labels = [
        ["id ref", "id obs"],
        ["iq ref", "iq obs"],
        ["Plant torque", "Observed torque"],
        ["Rotor speed"],
        ["Rotor flux magnitude"],
        ["vsd", "vsq"],
        ["Voltage saturation"],
    ],
    fig = "Hybrid FOC current loop",
    title = "Discrete FOC current loop + continuous IM plant",
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

    torque_Nm = collect(res.torque),
    torque_obs_Nm = collect(res.torque_obs),

    flux_r_Wb = collect(res.flux_r),
    theta_e_rad = collect(res.theta_e),
    omega_e_rad_s = collect(res.omega_e),

    Tload_Nm = collect(res.Tload),

    is_alpha_A = collect(res.isα),
    is_beta_A = collect(res.isβ),

    voltage_saturation = collect(res.saturation),
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
println("Final id ref = ", res.isd_ref[end], " A")
println("Final iq ref = ", res.isq_ref[end], " A")
println("Final id obs = ", res.isd[end], " A")
println("Final iq obs = ", res.isq[end], " A")
println("Final speed = ", res.speed_rpm[end], " rpm")
println("Final torque = ", res.torque[end], " N m")
println("Final observed torque = ", res.torque_obs[end], " N m")
println("Final rotor flux = ", res.flux_r[end], " Wb")
println()
println("Press ENTER to close.")
readline()