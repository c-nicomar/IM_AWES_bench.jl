using Pkg

# Run with:
#
#   julia --project=examples examples/run_foc_speed_f1_im_160kw_controlplots.jl
#
# or from a Julia session opened at the repository root:
#
#   include("examples/run_foc_speed_f1_im_160kw_controlplots.jl")

Pkg.activate(joinpath(@__DIR__))
Pkg.instantiate()

using IM_AWES_bench
using ControlPlots
using Printf
using Statistics

# ============================================================
# Run the simple bipolar speed/load test
# ============================================================

res = simulate_foc_speed_f1_im_160kw(
    t_end = 13.0,
    Ts = 100e-6,
    plant_substeps = 1,

    wm_ref_abs = 50.0,

    Tload_step1 = 200.0,
    Tload_step2 = 400.0,

    # First validation:
    # use the exact simulated load torque in the speed-controller
    # feedforward. Later change to :none or :kalman.
    load_estimator = :actual,
    use_load_feedforward = true,

    # At ±50 rad/s this should remain below the configured
    # field-weakening threshold.
    use_field_weakening = true,
)

# ============================================================
# Derived signals and summary metrics
# ============================================================

t = res.t

i_ref_mod = hypot.(res.isd_ref, res.isq_ref)
i_mod = hypot.(res.isd, res.isq)

speed_error = res.wm_ref_ramp .- res.omega_m

metric_mask = t .>= 1.0

speed_rmse = sqrt(mean(speed_error[metric_mask] .^ 2))
speed_peak_error = maximum(abs.(speed_error[metric_mask]))

current_saturation_fraction =
    mean(res.saturation_current .> 0.5)

torque_saturation_fraction =
    mean(abs.(res.sat_Te) .> 0.5)

q_current_saturation_fraction =
    mean(abs.(res.sat_isq) .> 0.5)

field_weakening_fraction =
    mean(res.field_weakening_active .> 0.5)

@printf("\n")
@printf("============================================================\n")
@printf("160 kW im IM: F1 speed-control test\n")
@printf("============================================================\n")
@printf("Speed RMSE after 1 s              : %10.4f rad/s\n", speed_rmse)
@printf("Peak absolute speed error         : %10.4f rad/s\n", speed_peak_error)
@printf("Maximum reference-current modulus : %10.2f A\n", maximum(i_ref_mod))
@printf("Maximum measured-current modulus  : %10.2f A\n", maximum(i_mod))
@printf("Maximum applied dq voltage        : %10.2f V\n", maximum(res.vs_mod))
@printf("Maximum unsaturated dq voltage    : %10.2f V\n", maximum(res.vs_mod_unsat))
@printf(
    "Current-controller saturation     : %10.3f %%\n",
    100 * current_saturation_fraction,
)
@printf(
    "Outer torque saturation           : %10.3f %%\n",
    100 * torque_saturation_fraction,
)
@printf(
    "Outer q-current saturation        : %10.3f %%\n",
    100 * q_current_saturation_fraction,
)
@printf(
    "Field-weakening active            : %10.3f %%\n",
    100 * field_weakening_fraction,
)
@printf("Nominal rotor time constant       : %10.6f s\n", res.nominal.tau_r)
@printf(
    "Nominal transient inductance      : %10.6e H\n",
    res.nominal.sigma_Lss,
)
@printf("============================================================\n\n")

# ============================================================
# Output folder
# ============================================================

output_dir = normpath(
    joinpath(@__DIR__, "..", "results", "im_160kw_f1"),
)

mkpath(output_dir)

# ============================================================
# Plot 1: speed and torque
#
# ControlPlots.plotx creates vertically aligned control channels.
# Each vector inside an argument is plotted in the same channel.
# ============================================================

p_speed_torque = plotx(
    t,

    [
        res.wm_ref,
        res.wm_ref_ramp,
        res.omega_m,
    ],

    [
        res.Tload,
        res.Te_ref_out,
        res.torque,
        res.torque_obs,
    ];

    xlabel = "Time [s]",

    ylabels = [
        "Mechanical speed [rad/s]",
        "Torque [N m]",
    ],

    labels = [
        [
            "Raw speed reference",
            "Ramped speed reference",
            "Machine speed",
        ],
        [
            "Load torque",
            "Electromagnetic torque reference",
            "Electromagnetic torque",
            "Observed electromagnetic torque",
        ],
    ],

    title = "160 kW im IM: speed and torque",
    fig = "im_160kw_speed_torque",
    legend_size = 9,
    loc = "best",
)

display(p_speed_torque)

ControlPlots.savefig(
    joinpath(output_dir, "speed_and_torque.png"),
)

# ============================================================
# Plot 2: dq currents, voltage utilization, and rotor flux
# ============================================================

voltage_limit = fill(res.ctrl_p.Vs_max, length(t))

p_electrical = plotx(
    t,

    [
        res.isd_ref,
        res.isd,
    ],

    [
        res.isq_ref,
        res.isq,
    ],

    [
        res.vs_mod,
        res.vs_mod_unsat,
        voltage_limit,
    ],

    [
        res.lambda_ref,
        res.flux_r,
    ];

    xlabel = "Time [s]",

    ylabels = [
        "d-axis current [A]",
        "q-axis current [A]",
        "dq voltage [V]",
        "Rotor flux [Wb]",
    ],

    labels = [
        [
            "isd reference",
            "isd observed",
        ],
        [
            "isq reference",
            "isq observed",
        ],
        [
            "Applied dq-voltage magnitude",
            "Unsaturated dq-voltage magnitude",
            "Voltage limit",
        ],
        [
            "Rotor-flux reference",
            "Observed rotor-flux magnitude",
        ],
    ],

    title = "160 kW im IM: electrical control variables",
    fig = "im_160kw_electrical",
    legend_size = 9,
    loc = "best",
)

display(p_electrical)

ControlPlots.savefig(
    joinpath(output_dir, "currents_voltage_flux.png"),
)

# ============================================================
# Plot 3: current magnitude, power, and saturation flags
# ============================================================

current_limit_inner =
    fill(res.ctrl_p.Is_max, length(t))

current_limit_outer =
    fill(res.outer_p.Is_max, length(t))

p_limits_power = plotx(
    t,

    [
        i_ref_mod,
        i_mod,
        current_limit_inner,
        current_limit_outer,
    ],

    [
        res.Pelec ./ 1e3,
        res.Pmech ./ 1e3,
        res.Pload ./ 1e3,
    ],

    [
        res.saturation_current,
        abs.(res.sat_Te),
        abs.(res.sat_isq),
        res.field_weakening_active,
    ];

    xlabel = "Time [s]",

    ylabels = [
        "dq current magnitude [A]",
        "Power [kW]",
        "Controller flag",
    ],

    labels = [
        [
            "Reference current magnitude",
            "Observed current magnitude",
            "Inner current limit",
            "Outer current limit",
        ],
        [
            "Electrical stator power",
            "Electromagnetic mechanical power",
            "External-load mechanical power",
        ],
        [
            "Voltage saturation",
            "Torque-reference saturation",
            "q-current saturation",
            "Field weakening",
        ],
    ],

    ylims = [
        nothing,
        nothing,
        (-0.1, 1.2),
    ],

    title = "160 kW im IM: limits and power",
    fig = "im_160kw_limits_power",
    legend_size = 9,
    loc = "best",
)

display(p_limits_power)

ControlPlots.savefig(
    joinpath(output_dir, "current_power_saturation.png"),
)

println("Saved figures to:")
println(output_dir)
