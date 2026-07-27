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

sol, sys = simulate_scalar_im(
    tspan = (0.0, 10.0),
    f_ref_val = 25.0,
    Tload_val = 0.0,
)

n_rpm = sys.n_rpm
Te = sys.Te

# Extra variables to store
ωm = sys.ωm
va = sys.va
vb = sys.vb
vc = sys.vc
vsα = sys.vsα
vsβ = sys.vsβ
isα = sys.isα
isβ = sys.isβ
irα = sys.irα
irβ = sys.irβ
ψsα = sys.ψsα
ψsβ = sys.ψsβ
ψrα = sys.ψrα
ψrβ = sys.ψrβ

f_ref = 25.0
p_pairs = 2.0
n_sync = 60 * f_ref / p_pairs

t = sol.t
speed = sol[n_rpm]
torque = sol[Te]
speed_sync = fill(n_sync, length(t))

p = plotx(
    t,
    [speed, speed_sync],
    torque;
    xlabel = "Time [s]",
    ylabels = ["Speed [rpm]", "Torque [N m]"],
    labels = [
        ["Rotor speed", "Synchronous speed"],
        ["Electromagnetic torque"],
    ],
    fig = "Scalar 25 Hz",
    title = "Scalar V/f induction machine - 25 Hz",
    yzoom = 1.25,
    legendsize = 9,
)

display(p)

# ------------------------------------------------------------
# Save simulation results as CSV
# ------------------------------------------------------------

results_dir = joinpath(@__DIR__, "..", "results")
mkpath(results_dir)

csv_file = joinpath(results_dir, "scalar_im_25Hz_results.csv")

df = DataFrame(
    t_s = collect(t),

    speed_rpm = collect(speed),
    speed_sync_rpm = collect(speed_sync),
    torque_Nm = collect(torque),
    omega_m_rad_s = collect(sol[ωm]),

    va_V = collect(sol[va]),
    vb_V = collect(sol[vb]),
    vc_V = collect(sol[vc]),
    vs_alpha_V = collect(sol[vsα]),
    vs_beta_V = collect(sol[vsβ]),

    is_alpha_A = collect(sol[isα]),
    is_beta_A = collect(sol[isβ]),
    ir_alpha_A = collect(sol[irα]),
    ir_beta_A = collect(sol[irβ]),

    psi_s_alpha_Wb = collect(sol[ψsα]),
    psi_s_beta_Wb = collect(sol[ψsβ]),
    psi_r_alpha_Wb = collect(sol[ψrα]),
    psi_r_beta_Wb = collect(sol[ψrβ]),
)

CSV.write(csv_file, df)

println("Saved simulation results to:")
println(csv_file)

println("Final speed = ", speed[end], " rpm")
println("Expected synchronous speed = ", n_sync, " rpm")
if ! isinteractive()
    println("Press ENTER to close.")
    readline()
end
