using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using InductionMachineDrives
# This script uses the MTK system builders, which live in a package
# extension. Both of these are needed to trigger it.
using ModelingToolkit
using OrdinaryDiffEq
using MakieControlPlots
using CSV
using DataFrames

# ============================================================
# Simulation case definition
# ============================================================

case_name = "scalar_frequency_steps_load_steps_with_observer"

# This script is a simulation scenario:
#
# Controller:
#   scalar V/f control
#
# Frequency input:
#   0 Hz for 1 s, then steps of 5 Hz until 25 Hz
#
# Mechanical load:
#   0 Nm until 6 s, 40 Nm from 6 to 8 s, 80 Nm after 8 s
#
# Plant:
#   induction machine in stationary alpha-beta coordinates
#
# Estimator:
#   rotor-flux and torque observer based on measured alpha-beta currents
#   and mechanical rotor angle

# ------------------------------------------------------------
# Simulation time
# ------------------------------------------------------------

t_start = 0.0
t_end = 12.0
tspan = (t_start, t_end)

# ------------------------------------------------------------
# Scalar control / frequency profile
# ------------------------------------------------------------

frequency_profile = :steps

f_ref = 25.0          # final frequency [Hz]
f_step = 5.0          # frequency increment [Hz]
t_hold = 1.0          # initial time at 0 Hz [s]
t_step = 1.0          # time between frequency steps [s]

# ------------------------------------------------------------
# Load torque profile
# ------------------------------------------------------------

load_profile = :steps

Tload_step1 = 40.0    # first load torque level [N m]
Tload_step2 = 80.0    # second load torque level [N m]
t_load_step1 = 6.0    # time of first load step [s]
t_load_step2 = 8.0    # time of second load step [s]

# ------------------------------------------------------------
# Observer settings
# ------------------------------------------------------------

include_observer = true

# From the MATLAB observer:
# tau_r = Lrr / Rr
# Current default from the MATLAB function was 0.3869971696 s.
tau_r = 0.3869971696

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

sol, sys = simulate_scalar_im(
    tspan = tspan,

    frequency_profile = frequency_profile,
    f_ref_val = f_ref,
    f_step_val = f_step,
    t_hold_val = t_hold,
    t_step_val = t_step,

    load_profile = load_profile,
    Tload_step1_val = Tload_step1,
    Tload_step2_val = Tload_step2,
    t_load_step1_val = t_load_step1,
    t_load_step2_val = t_load_step2,

    p_val = p_pairs,

    include_observer = include_observer,
    tau_r_val = tau_r,
)

# ============================================================
# Extract variables
# ============================================================

t = sol.t

# ------------------------------------------------------------
# Profile / reference signals
# ------------------------------------------------------------

f_cmd = sol[sys.f_cmd]
Tload_cmd = sol[sys.Tload_cmd]

# ------------------------------------------------------------
# Main plant outputs
# ------------------------------------------------------------

speed = sol[sys.n_rpm]
speed_sync = sol[sys.n_sync_rpm]
torque = sol[sys.Te]
ωm = sol[sys.ωm]

# ------------------------------------------------------------
# Controller voltage outputs
# ------------------------------------------------------------

va = sol[sys.va]
vb = sol[sys.vb]
vc = sol[sys.vc]

vsα = sol[sys.vsα]
vsβ = sol[sys.vsβ]

# ------------------------------------------------------------
# Plant electrical variables
# ------------------------------------------------------------

isα = sol[sys.isα]
isβ = sol[sys.isβ]

irα = sol[sys.irα]
irβ = sol[sys.irβ]

ψsα = sol[sys.ψsα]
ψsβ = sol[sys.ψsβ]

ψrα = sol[sys.ψrα]
ψrβ = sol[sys.ψrβ]

# ------------------------------------------------------------
# Observer variables
# ------------------------------------------------------------

if include_observer
    Te_obs = sol[sys.Te_obs]

    flux_r_mod_obs = sol[sys.flux_r_mod_obs]
    θe_obs = sol[sys.θe_obs]

    isd_e_obs = sol[sys.isd_e_obs]
    isq_e_obs = sol[sys.isq_e_obs]

    λrd_e_obs = sol[sys.λrd_e_obs]
    λrq_e_obs = sol[sys.λrq_e_obs]

    ψsd_e_obs = sol[sys.ψsd_e_obs]
    ψsq_e_obs = sol[sys.ψsq_e_obs]
    flux_s_mod_obs = sol[sys.flux_s_mod_obs]
end

# ============================================================
# Plot
# ============================================================

if include_observer

    p_plot = plotx(
        t,
        f_cmd,
        [speed, speed_sync],
        [torque, Tload_cmd, Te_obs],
        flux_r_mod_obs,
        [isd_e_obs, isq_e_obs];
        xlabel = "Time [s]",
        ylabels = [
            "Frequency [Hz]",
            "Speed [rpm]",
            "Torque [N m]",
            "Rotor flux [Wb]",
            "Current [A]",
        ],
        labels = [
            ["Frequency command"],
            ["Rotor speed", "Synchronous speed"],
            ["Plant torque", "Load torque", "Observed torque"],
            ["Estimated rotor flux magnitude"],
            ["i_sd,e obs", "i_sq,e obs"],
        ],
        fig = "Scalar frequency/load steps with observer",
        title = "Scalar V/f control with load steps and rotor-flux observer",
        yzoom = 1.25,
        legendsize = 14,
    )

else

    p_plot = plotx(
        t,
        f_cmd,
        [speed, speed_sync],
        [torque, Tload_cmd];
        xlabel = "Time [s]",
        ylabels = [
            "Frequency [Hz]",
            "Speed [rpm]",
            "Torque [N m]",
        ],
        labels = [
            ["Frequency command"],
            ["Rotor speed", "Synchronous speed"],
            ["Electromagnetic torque", "Load torque"],
        ],
        fig = "Scalar frequency/load steps",
        title = "Scalar V/f control: frequency steps followed by load-torque steps",
        yzoom = 1.25,
        legendsize = 14,
    )

end

display(p_plot)

# ============================================================
# Save results
# ============================================================

df = DataFrame(
    t_s = collect(t),

    # Profiles / references
    f_cmd_Hz = collect(f_cmd),
    Tload_cmd_Nm = collect(Tload_cmd),

    # Main plant outputs
    speed_rpm = collect(speed),
    speed_sync_rpm = collect(speed_sync),
    torque_Nm = collect(torque),
    omega_m_rad_s = collect(ωm),

    # Controller voltages
    va_V = collect(va),
    vb_V = collect(vb),
    vc_V = collect(vc),

    vs_alpha_V = collect(vsα),
    vs_beta_V = collect(vsβ),

    # Plant currents
    is_alpha_A = collect(isα),
    is_beta_A = collect(isβ),

    ir_alpha_A = collect(irα),
    ir_beta_A = collect(irβ),

    # Plant fluxes
    psi_s_alpha_Wb = collect(ψsα),
    psi_s_beta_Wb = collect(ψsβ),

    psi_r_alpha_Wb = collect(ψrα),
    psi_r_beta_Wb = collect(ψrβ),
)

if include_observer
    df.Te_obs_Nm = collect(Te_obs)

    df.flux_r_mod_obs_Wb = collect(flux_r_mod_obs)
    df.theta_e_obs_rad = collect(θe_obs)

    df.isd_e_obs_A = collect(isd_e_obs)
    df.isq_e_obs_A = collect(isq_e_obs)

    df.lambda_rd_e_obs_Wb = collect(λrd_e_obs)
    df.lambda_rq_e_obs_Wb = collect(λrq_e_obs)

    df.psi_sd_e_obs_Wb = collect(ψsd_e_obs)
    df.psi_sq_e_obs_Wb = collect(ψsq_e_obs)
    df.flux_s_mod_obs_Wb = collect(flux_s_mod_obs)
end

CSV.write(csv_file, df)

# ============================================================
# Console summary
# ============================================================

println("Saved simulation results to:")
println(csv_file)

println()
println("Simulation case: ", case_name)
println("Simulation time: ", t_start, " to ", t_end, " s")
println("Frequency profile: ", frequency_profile)
println("Load profile: ", load_profile)
println("Observer enabled: ", include_observer)
println()
println("Final frequency command = ", f_cmd[end], " Hz")
println("Final load torque = ", Tload_cmd[end], " N m")
println("Final speed = ", speed[end], " rpm")
println("Final synchronous speed = ", speed_sync[end], " rpm")
println("Final electromagnetic torque = ", torque[end], " N m")

if include_observer
    println("Final observed torque = ", Te_obs[end], " N m")
    println("Final observed rotor flux = ", flux_r_mod_obs[end], " Wb")
end
