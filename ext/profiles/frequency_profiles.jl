"""
    build_frequency_command_eqs(f_cmd; frequency_profile, f_ref, f_ref_val,
                                f_step_val, t_hold_val, t_step_val)

Builds the frequency reference profile equations.

This block is a test/reference generator, not a controller and not the plant.

Supported profiles:

- `:constant`
- `:steps`

For `:steps`, with:

    f_ref_val = 25.0
    f_step_val = 5.0
    t_hold_val = 1.0
    t_step_val = 1.0

the profile is:

    0–1 s      -> 0 Hz
    1–2 s      -> 5 Hz
    2–3 s      -> 10 Hz
    3–4 s      -> 15 Hz
    4–5 s      -> 20 Hz
    5 s onward -> 25 Hz
"""
function build_frequency_command_eqs(
    f_cmd;
    frequency_profile,
    f_ref,
    f_ref_val,
    f_step_val,
    t_hold_val,
    t_step_val,
)

    if frequency_profile == :constant

        f_cmd_expr = f_ref

    elseif frequency_profile == :steps

        if f_step_val <= 0.0
            error("f_step_val must be positive when frequency_profile = :steps.")
        end

        if t_step_val <= 0.0
            error("t_step_val must be positive when frequency_profile = :steps.")
        end

        if f_ref_val < 0.0
            error("f_ref_val must be non-negative.")
        end

        n_steps = Int(ceil(f_ref_val / f_step_val))

        # Default value after the final step
        f_cmd_expr = f_ref

        # Build nested ifelse expression backwards.
        #
        # Example:
        # t < 1 -> 0 Hz
        # t < 2 -> 5 Hz
        # t < 3 -> 10 Hz
        # t < 4 -> 15 Hz
        # t < 5 -> 20 Hz
        # else  -> 25 Hz
        for k in (n_steps - 1):-1:0
            threshold = t_hold_val + k * t_step_val
            f_k = min(k * f_step_val, f_ref_val)
            f_cmd_expr = ifelse(t < threshold, f_k, f_cmd_expr)
        end

    else

        error("Unknown frequency_profile = $frequency_profile. Use :constant or :steps.")

    end

    return [
        f_cmd ~ f_cmd_expr,
    ]
end


"""
    frequency_profile_tstops(; frequency_profile, f_ref_val, f_step_val,
                              t_hold_val, t_step_val, tspan)

Returns solver stop times for discontinuities in the frequency profile.
"""
function frequency_profile_tstops(;
    frequency_profile,
    f_ref_val,
    f_step_val,
    t_hold_val,
    t_step_val,
    tspan,
)

    tstops = Float64[]

    if frequency_profile == :steps
        n_steps = Int(ceil(f_ref_val / f_step_val))

        frequency_step_times = [
            t_hold_val + k * t_step_val
            for k in 0:n_steps
        ]

        append!(tstops, frequency_step_times)
    end

    return filter(τ -> tspan[1] < τ < tspan[2], tstops)
end