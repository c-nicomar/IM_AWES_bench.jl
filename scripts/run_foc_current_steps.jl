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

case_name = "foc_current_steps"

# This script tests only the inner current loop of a first FOC structure.
#
# References:
#   0–1 s      id_ref = 0 A,  iq_ref = 0 A
#   1–3 s      id_ref = 10 A, iq_ref = 0 A
#   3–6 s      id_ref = 10 A, iq_ref = 15 A
#   6–9 s      id_ref = 10 A, iq_ref = -15 A
#   9 s onward id_ref = 10 A, iq_ref = 0 A
#
# Expected behavior:
#   id should build rotor flux.
#   positive iq should accelerate the machine.
#   negative iq should decelerate the machine.

# ------------------------------------------------------------
# Simulation time
# ------------------------------------------------------------

t_start = 0.0
t_end = 12.0
tspan = (t_start, t_end)

# ------------------------------------------------------------
# Current reference profile
# ------------------------------------------------------------

current_reference_profile = :steps

t_id_step = 1.0
t_iq_pos_step = 3.0
t_iq_neg_step = 6.0
t_iq_zero_step = 9.0

isd_ref_mag = 10.0
isq_ref_pos = 15.0
isq_ref_neg = -15.0

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

# ------------------------------------------------------------
# Machine parameters explicitly changed in this scenario
# ------------------------------------------------------------

p_pairs = 2.0

# ------------------------------------------------------------
# Output settings
# ------------------------------------------------------------

results_dir = joinpath(@__DIR__, "..", "results")
mkpath(results_dir)

csv_file = joinpath(results_dir, case_name * ".csv")

# ============================================================
# Run simulation
# ============================================================

sol, sys = simulate_foc_current_im(
    tspan = tspan,

    current_reference_profile = current_reference_profile,
    t_id_step_val = t_id_step,
    t_iq_pos_step_val = t_iq_pos_step,
    t_iq_neg_step_val = t_iq_neg_step,
    t_iq_zero_step_val = t_iq_zero_step,

    isd_ref_mag_val = isd_ref_mag,
    isq_ref_pos_val = isq_ref_pos,
    isq_ref_neg_val = isq_ref_neg,

    load_profile = load_profile,
    Tload_val = Tload,

    Vs_max_val = Vs_max,
    Is_max_val = Is_max,

    p_val = p_pairs,
)

# ============================================================
# Extract variables
# ============================================================

t = sol.t

# References
isd_ref = sol[sys.isd_ref]
isq_ref = sol[sys.isq_ref]

isd_ref_lim = sol[sys.isd_ref_lim]
isq_ref_lim = sol[sys.isq_ref_lim]

# Observer current measurements
isd = sol[sys.isd_e_obs]
isq = sol[sys.isq_e_obs]

# Controller outputs
vsd = sol[sys.vsd_ref]
vsq = sol[sys.vsq_ref]
vs_mod = sol[sys.vs_mod_unsat]
sat = sol[sys.voltage_saturation]

# Plant outputs
speed = sol[sys.n_rpm]
torque = sol[sys.Te]
Te_obs = sol[sys.Te_obs]

# Observer outputs
flux_r = sol[sys.flux_r_mod_obs]
theta_e = sol[sys.θe_obs]
ws_obs = sol[sys.ws_obs]

# Plant alpha-beta currents/voltages
isα = sol[sys.isα]
isβ = sol[sys.isβ]
vsα = sol[sys.vsα]
vsβ = sol[sys.vsβ]

# ============================================================
# Plot
# ============================================================

p_plot = plotx(
    t,
    [isd_ref, isd],
    [isq_ref, isq],
    [torque, Te_obs],
    speed,
    flux_r,
    [vsd, vsq],
    sat;
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
        ["id ref", "id measured"],
        ["iq ref", "iq measured"],
        ["Plant torque", "Observed torque"],
        ["Rotor speed"],
        ["Rotor flux magnitude"],
        ["vsd", "vsq"],
        ["Voltage saturation"],
    ],
    fig = "FOC current loop test",
    title = "FOC inner current loop test",
    yzoom = 1.20,
    legendsize = 8,
)

display(p_plot)

# ============================================================
# Save results
# ============================================================

df = DataFrame(
    t_s = collect(t),

    isd_ref_A = collect(isd_ref),
    isq_ref_A = collect(isq_ref),

    isd_ref_lim_A = collect(isd_ref_lim),
    isq_ref_lim_A = collect(isq_ref_lim),

    isd_obs_A = collect(isd),
    isq_obs_A = collect(isq),

    vsd_ref_V = collect(vsd),
    vsq_ref_V = collect(vsq),
    vs_mod_unsat_V = collect(vs_mod),
    voltage_saturation = collect(sat),

    speed_rpm = collect(speed),
    torque_Nm = collect(torque),
    Te_obs_Nm = collect(Te_obs),

    flux_r_mod_obs_Wb = collect(flux_r),
    theta_e_obs_rad = collect(theta_e),
    ws_obs_rad_s = collect(ws_obs),

    is_alpha_A = collect(isα),
    is_beta_A = collect(isβ),

    vs_alpha_V = collect(vsα),
    vs_beta_V = collect(vsβ),
)

CSV.write(csv_file, df)

# ============================================================
# Console summary
# ============================================================

println("Saved simulation results to:")
println(csv_file)

println()
println("Simulation case: ", case_name)
println("Simulation time: ", t_start, " to ", t_end, " s")
println("Current reference profile: ", current_reference_profile)
println("Load profile: ", load_profile)
println()
println("Final id ref = ", isd_ref[end], " A")
println("Final iq ref = ", isq_ref[end], " A")
println("Final id measured = ", isd[end], " A")
println("Final iq measured = ", isq[end], " A")
println("Final speed = ", speed[end], " rpm")
println("Final torque = ", torque[end], " N m")
println("Final observed torque = ", Te_obs[end], " N m")
println("Final rotor flux = ", flux_r[end], " Wb")
