using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using IM_AWES_bench
# This script uses the MTK system builders, which live in a package
# extension. Both of these are needed to trigger it.
using ModelingToolkit
using OrdinaryDiffEq
using MakieControlPlots
using CSV
using DataFrames

# ------------------------------------------------------------
# Simulation setup
# ------------------------------------------------------------

f_ref = 25.0
f_step = 5.0
t_hold = 1.0
t_step = 1.0
p_pairs = 2.0
Tload = 0.0

sol, sys = simulate_scalar_im(
    tspan = (0.0, 10.0),
    frequency_profile = :steps,
    f_ref_val = f_ref,
    f_step_val = f_step,
    t_hold_val = t_hold,
    t_step_val = t_step,
    Tload_val = Tload,
)

# ------------------------------------------------------------
# Extract variables
# ------------------------------------------------------------

t = sol.t

f_cmd = sol[sys.f_cmd]
n_sync = sol[sys.n_sync_rpm]
speed = sol[sys.n_rpm]
torque = sol[sys.Te]

ωm = sol[sys.ωm]

va = sol[sys.va]
vb = sol[sys.vb]
vc = sol[sys.vc]
vsα = sol[sys.vsα]
vsβ = sol[sys.vsβ]

isα = sol[sys.isα]
isβ = sol[sys.isβ]
irα = sol[sys.irα]
irβ = sol[sys.irβ]

ψsα = sol[sys.ψsα]
ψsβ = sol[sys.ψsβ]
ψrα = sol[sys.ψrα]
ψrβ = sol[sys.ψrβ]

# ------------------------------------------------------------
# Plot
# ------------------------------------------------------------

p_plot = plotx(
    t,
    f_cmd,
    [speed, n_sync],
    torque;
    xlabel = "Time [s]",
    ylabels = [
        "Frequency [Hz]",
        "Speed [rpm]",
        "Torque [N m]",
    ],
    labels = [
        ["Frequency command"],
        ["Rotor speed", "Synchronous speed"],
        ["Electromagnetic torque"],
    ],
    fig = "Scalar frequency steps",
    title = "Scalar V/f control with stepped frequency reference",
    yzoom = 1.25,
    legendsize = 9,
)

display(p_plot)

# ------------------------------------------------------------
# Save simulation results as CSV
# ------------------------------------------------------------

results_dir = joinpath(@__DIR__, "..", "results")
mkpath(results_dir)

csv_file = joinpath(results_dir, "scalar_frequency_steps_results.csv")

df = DataFrame(
    t_s = collect(t),

    f_cmd_Hz = collect(f_cmd),

    speed_rpm = collect(speed),
    speed_sync_rpm = collect(n_sync),
    torque_Nm = collect(torque),
    omega_m_rad_s = collect(ωm),

    va_V = collect(va),
    vb_V = collect(vb),
    vc_V = collect(vc),
    vs_alpha_V = collect(vsα),
    vs_beta_V = collect(vsβ),

    is_alpha_A = collect(isα),
    is_beta_A = collect(isβ),
    ir_alpha_A = collect(irα),
    ir_beta_A = collect(irβ),

    psi_s_alpha_Wb = collect(ψsα),
    psi_s_beta_Wb = collect(ψsβ),
    psi_r_alpha_Wb = collect(ψrα),
    psi_r_beta_Wb = collect(ψrβ),
)

CSV.write(csv_file, df)

println("Saved simulation results to:")
println(csv_file)

println()
println("Final frequency command = ", f_cmd[end], " Hz")
println("Final speed = ", speed[end], " rpm")
println("Final synchronous speed = ", n_sync[end], " rpm")
println("Final torque = ", torque[end], " N m")
println()
println("Press ENTER to close.")
readline()