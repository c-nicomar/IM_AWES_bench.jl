using Pkg

# Run with:
#
#   julia --project=scripts scripts/run_foc_speed_f1_160kw.jl
#
# or from a Julia session opened at the repository root:
#
#   include("scripts/run_foc_speed_f1_160kw.jl")

if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using IM_AWES_bench
using MakieControlPlots
using Printf
using Statistics

# ============================================================
# Run the simple bipolar speed/load test
# ============================================================

res = simulate_foc_speed_f1_im_160kw(
    t_end = 13.0,
    Ts = 100e-6,
    plant_substeps = 1,

    wm_ref_abs = 170.0,

    Tload_step1 = 200.0,
    Tload_step2 = 400.0,

    # First validation:
    # use the exact simulated load torque in the speed-controller
    # feedforward. Later change to :none or :kalman.
    load_estimator = :kalman,
    use_load_feedforward = true,

    # At ±170 rad/s this crosses the configured field-weakening threshold.
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

# Verify that field weakening is driven by the observer's synchronous
# electrical speed, rather than only by pole-pair-scaled mechanical speed.
omega_e_mechanical = res.outer_p.p .* res.omega_m
omega_sl_est = res.omega_e .- omega_e_mechanical
omega_e_base_fw = res.outer_p.p * res.outer_p.wm_base_fw
field_weakening_expected = abs.(res.omega_e) .> omega_e_base_fw
field_weakening_actual = res.field_weakening_active .> 0.5
field_weakening_mismatches = count(field_weakening_expected .!= field_weakening_actual)

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
@printf("Maximum |estimated omega_e|        : %10.3f rad/s\n", maximum(abs.(res.omega_e)))
@printf("Maximum |estimated slip speed|     : %10.3f rad/s\n", maximum(abs.(omega_sl_est)))
@printf("Electrical FW threshold            : %10.3f rad/s\n", omega_e_base_fw)
@printf("FW threshold/flag mismatches       : %10d samples\n", field_weakening_mismatches)
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

# Older MakieControlPlots releases could fail in the internal legend-row
# predictor used by `plotx` when several stacked axes have legends. Toggle
# this to false to fall back to a manual GLMakie plotter if that resurfaces
# with the currently installed version.
const USE_PLOTX = true

const GLM = MakieControlPlots.GLMakie
const PLOT_SCREENS = Any[]

function display_in_new_screen(figure)
    screen = GLM.Screen()
    display(screen, figure)
    push!(PLOT_SCREENS, screen)
    return screen
end

function plot_channels(
    x,
    channels...;
    xlabel,
    ylabels,
    labels,
    ylims = nothing,
    title = "",
    fig = "",
    legendsize = 11,
    legend_position = :rt,
    yzoom = 1.5,
)
    n = length(channels)
    figure = GLM.Figure(size = (1000, round(Int, 260 * yzoom * n)))
    axes = GLM.Axis[]

    for i in 1:n
        ax = GLM.Axis(
            figure[i, 1],
            title = i == 1 ? title : "",
            xlabel = i == n ? xlabel : "",
            ylabel = ylabels[i],
        )
        push!(axes, ax)

        for (series, label) in zip(channels[i], labels[i])
            GLM.lines!(ax, x, series; label)
        end
        GLM.axislegend(ax; position = legend_position, labelsize = legendsize)

        if !isnothing(ylims) && !isnothing(ylims[i])
            GLM.ylims!(ax, ylims[i]...)
        end
        i < n && GLM.hidexdecorations!(ax; grid = false)
    end

    n > 1 && GLM.linkxaxes!(axes...)
    return figure
end

function make_plot(
    x,
    channels...;
    xlabel,
    ylabels,
    labels,
    ylims = nothing,
    title = "",
    fig = "",
    legendsize = 11,
    legend_position = :rt,
    yzoom = 1.5,
)
    if USE_PLOTX
        return MakieControlPlots.plotx(
            x,
            channels...;
            xlabel,
            ylabels,
            labels,
            ylims,
            title,
            fig,
            legendsize,
            legend_position,
            yzoom,
            disp = true,
            new_screen = true,
        )
    else
        figure = plot_channels(
            x,
            channels...;
            xlabel,
            ylabels,
            labels,
            ylims,
            title,
            fig,
            legendsize,
            legend_position,
            yzoom,
        )
        display_in_new_screen(figure)
        return figure
    end
end

# ============================================================
# Plot 1: speed and torque
#
# Each vector inside an argument is plotted in the same GLMakie axis.
# ============================================================

p_speed_torque = make_plot(
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
    legendsize = 11,
    legend_position = :rt,
    yzoom = 1.5,
)



# ============================================================
# Plot 2: dq currents, voltage utilization, and rotor flux
# ============================================================

voltage_limit = fill(res.ctrl_p.Vs_max, length(t))

p_electrical = make_plot(
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
    legendsize = 11,
    legend_position = :rt,
    yzoom = 1.5,
)

# ============================================================
# Plot 3: current magnitude, power, and saturation flags
# ============================================================

current_limit_inner =
    fill(res.ctrl_p.Is_max, length(t))

current_limit_outer =
    fill(res.outer_p.Is_max, length(t))

p_limits_power = make_plot(
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
    legendsize = 11,
    legend_position = :rt,
    yzoom = 1.5,
)

# ============================================================
# Plot 4: estimated synchronous speed used by field weakening
# ============================================================

omega_e_base_positive = fill(omega_e_base_fw, length(t))
omega_e_base_negative = fill(-omega_e_base_fw, length(t))

p_omega_e = make_plot(
    t,

    [
        res.omega_e,
        omega_e_mechanical,
        omega_e_base_positive,
        omega_e_base_negative,
    ],

    [
        Float64.(field_weakening_expected),
        Float64.(field_weakening_actual),
    ];

    xlabel = "Time [s]",

    ylabels = [
        "Electrical speed [rad/s]",
        "Field weakening flag",
    ],

    labels = [
        [
            "Estimated synchronous omega_e",
            "p * mechanical omega_m",
            "+ electrical FW threshold",
            "- electrical FW threshold",
        ],
        [
            "Expected from |omega_e| threshold",
            "Outer-controller FW flag",
        ],
    ],

    ylims = [
        nothing,
        (-0.1, 1.2),
    ],

    title = "160 kW im IM: estimator-based field weakening",
    fig = "im_160kw_estimated_omega_e",
    legendsize = 11,
    legend_position = :rt,
    yzoom = 1.5,
)


