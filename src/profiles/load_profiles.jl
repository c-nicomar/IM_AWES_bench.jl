"""
    build_load_torque_eqs(Tload_cmd; load_profile, Tload,
                          Tload_step1_val, Tload_step2_val,
                          t_load_step1_val, t_load_step2_val)

Builds the load torque profile equations.

This block is a mechanical input generator, not the plant.

Supported profiles:

- `:constant`
- `:steps`

For `:steps`, the profile is:

    0–t_load_step1       -> 0 Nm
    t_load_step1–t_load_step2 -> Tload_step1_val
    after t_load_step2   -> Tload_step2_val
"""
function build_load_torque_eqs(
    Tload_cmd;
    load_profile,
    Tload,
    Tload_step1_val,
    Tload_step2_val,
    t_load_step1_val,
    t_load_step2_val,
)

    if load_profile == :constant

        Tload_cmd_expr = Tload

    elseif load_profile == :steps

        if t_load_step2_val <= t_load_step1_val
            error("t_load_step2_val must be larger than t_load_step1_val.")
        end

        Tload_cmd_expr = ifelse(
            t < t_load_step1_val,
            0.0,
            ifelse(
                t < t_load_step2_val,
                Tload_step1_val,
                Tload_step2_val,
            ),
        )

    else

        error("Unknown load_profile = $load_profile. Use :constant or :steps.")

    end

    return [
        Tload_cmd ~ Tload_cmd_expr,
    ]
end


"""
    load_profile_tstops(; load_profile, t_load_step1_val,
                         t_load_step2_val, tspan)

Returns solver stop times for discontinuities in the load profile.
"""
function load_profile_tstops(;
    load_profile,
    t_load_step1_val,
    t_load_step2_val,
    tspan,
)

    tstops = Float64[]

    if load_profile == :steps
        append!(tstops, [t_load_step1_val, t_load_step2_val])
    end

    return filter(τ -> tspan[1] < τ < tspan[2], tstops)
end