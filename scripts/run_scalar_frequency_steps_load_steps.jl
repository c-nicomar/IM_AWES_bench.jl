using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))

include("../src/IM_AWES_bench_jl.jl")

using ControlPlots
using CSV
using DataFrames

# ============================================================
# Simulation case definition
# ============================================================

case_name = "scalar_frequency_steps_load_steps"

# Simulation time
t_start = 0.0
t_end = 12.0
tspan = (t_start, t_end)

# Scalar control settings
frequency_profile = :steps
f_ref = 25.0          # final frequency [Hz]
f_step = 5.0          # frequency increment [Hz]
t_hold = 1.0          # initial time at 0 Hz [s]
t_step = 1.0          # time between frequency steps [s]

# Load torque settings
load_profile = :steps
Tload_step1 = 40.0    # first load torque level [N m]
Tload_step2 = 80.0    # second load torque level [N m]
t_load_step1 = 6.0    # time of first load step [s]
t_load_step2 = 8.0    # time of second load step [s]

# Machine parameters
p_pairs = 2.0

# Output settings
results_dir = joinpath(@__DIR__, "..", "results")
mkpath(results_dir)

csv_file = joinpath(results_dir, case_name * ".csv")

# ============================================================
# Run simulation
# ============================================================

sol, sys = IM_AWES_bench_jl.simulate_scalar_im(
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
)

# ============================================================
# Extract variables
# ============================================================

t = sol.t

f_cmd = sol[sys.f_cmd]
Tload_cmd = sol[sys.Tload_cmd]

speed = sol[sys.n_rpm]
speed_sync = sol[sys.n_sync_rpm]
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

# ============================================================
# Plot
# ============================================================

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
    fig = "Scalar frequency steps with load steps",
    title = "Scalar V/f control: frequency steps followed by load-torque steps",
    yzoom = 1.25,
    legend_size = 9,
    loc = "best",
)

display(p_plot)

# ============================================================
# Save results
# ============================================================

df = DataFrame(
    t_s = collect(t),

    f_cmd_Hz = collect(f_cmd),
    Tload_cmd_Nm = collect(Tload_cmd),

    speed_rpm = collect(speed),
    speed_sync_rpm = collect(speed_sync),
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
println("Simulation case: ", case_name)
println("Simulation time: ", t_start, " to ", t_end, " s")
println("Frequency profile: ", frequency_profile)
println("Load profile: ", load_profile)
println()
println("Final frequency command = ", f_cmd[end], " Hz")
println("Final load torque = ", Tload_cmd[end], " N m")
println("Final speed = ", speed[end], " rpm")
println("Final synchronous speed = ", speed_sync[end], " rpm")
println("Final electromagnetic torque = ", torque[end], " N m")
println()
println("Press ENTER to close.")
readline()