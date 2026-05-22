module IM_AWES_bench_jl

using ModelingToolkit
using ModelingToolkit: t_nounits as t, D_nounits as D
using OrdinaryDiffEq

export build_scalar_im_model, simulate_scalar_im

"""
    build_scalar_im_model(; kwargs...)

Builds a simplified squirrel-cage induction machine model driven by open-loop scalar V/f control.

This first version uses stationary αβ equations, not full Simscape-style electrical pins.

Frequency profile options:

- `frequency_profile = :constant`
- `frequency_profile = :steps`

Load torque profile options:

- `load_profile = :constant`
- `load_profile = :steps`
"""
function build_scalar_im_model(;
    # Machine parameters
    Rs_val = 0.21946,
    Rr_val = 0.11659,
    Lls_val = 4.28e-3,
    Llr_val = 4.28e-3,
    Lm_val = 40.84e-3,
    p_val = 2.0,
    J_val = 0.2685 + 0.1,
    B_val = 0.01298,

    # Scalar-control parameters
    Vll_nom_val = 380.0,
    f_nom_val = 50.0,
    f_ref_val = 25.0,
    frequency_profile = :constant,
    f_step_val = 5.0,
    t_hold_val = 1.0,
    t_step_val = 1.0,

    # Mechanical load parameters
    Tload_val = 0.0,
    load_profile = :constant,
    Tload_step1_val = 40.0,
    Tload_step2_val = 80.0,
    t_load_step1_val = 6.0,
    t_load_step2_val = 8.0,
)

    @parameters begin
        Rs = Rs_val
        Rr = Rr_val
        Lls = Lls_val
        Llr = Llr_val
        Lm = Lm_val
        p = p_val
        J = J_val
        B = B_val

        Vll_nom = Vll_nom_val
        f_nom = f_nom_val
        f_ref = f_ref_val

        Tload = Tload_val
    end

    @variables begin
        # Scalar-control variables
        θs(t) = 0.0
        f_cmd(t)
        Vphase_peak(t)

        # Electrical state variables
        isα(t) = 0.0
        isβ(t) = 0.0
        irα(t) = 0.0
        irβ(t) = 0.0

        # Mechanical state variables
        ωm(t) = 0.0
        θm(t) = 0.0

        # Flux linkages
        ψsα(t)
        ψsβ(t)
        ψrα(t)
        ψrβ(t)

        # Voltage variables
        va(t)
        vb(t)
        vc(t)
        vsα(t)
        vsβ(t)

        # Outputs
        Te(t)
        Tload_cmd(t)
        n_rpm(t)
        n_sync_rpm(t)
    end

    Ls = Lls + Lm
    Lr = Llr + Lm

    # ------------------------------------------------------------
    # Frequency command expression
    # ------------------------------------------------------------

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
            error("f_ref_val must be non-negative in this first implementation.")
        end

        n_steps = Int(ceil(f_ref_val / f_step_val))

        # Default after the last step
        f_cmd_expr = f_ref

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

    # ------------------------------------------------------------
    # Load torque command expression
    # ------------------------------------------------------------

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

    eqs = [
        # Frequency command
        f_cmd ~ f_cmd_expr,

        # Load torque command
        Tload_cmd ~ Tload_cmd_expr,

        # Scalar V/f control
        D(θs) ~ 2π * f_cmd,

        Vphase_peak ~ sqrt(2) * Vll_nom / sqrt(3) * f_cmd / f_nom,

        va ~ Vphase_peak * sin(θs),
        vb ~ Vphase_peak * sin(θs - 2π / 3),
        vc ~ Vphase_peak * sin(θs + 2π / 3),

        # abc to stationary alpha-beta
        vsα ~ 2 / 3 * (va - 0.5 * vb - 0.5 * vc),
        vsβ ~ 2 / 3 * (sqrt(3) / 2 * (vb - vc)),

        # Flux-current relations
        ψsα ~ Ls * isα + Lm * irα,
        ψsβ ~ Ls * isβ + Lm * irβ,
        ψrα ~ Lm * isα + Lr * irα,
        ψrβ ~ Lm * isβ + Lr * irβ,

        # Stator voltage equations
        vsα ~ Rs * isα + D(ψsα),
        vsβ ~ Rs * isβ + D(ψsβ),

        # Rotor squirrel-cage equations in stationary alpha-beta frame
        0 ~ Rr * irα + D(ψrα) + p * ωm * ψrβ,
        0 ~ Rr * irβ + D(ψrβ) - p * ωm * ψrα,

        # Electromagnetic torque
        Te ~ 1.5 * p * (ψsα * isβ - ψsβ * isα),

        # Mechanical dynamics
        J * D(ωm) ~ Te - Tload_cmd - B * ωm,
        D(θm) ~ ωm,

        # Useful outputs
        n_rpm ~ ωm * 60 / (2π),
        n_sync_rpm ~ 60 * f_cmd / p,
    ]

    @named sys = ODESystem(eqs, t)
    return structural_simplify(sys)
end


"""
    simulate_scalar_im(; tspan=(0.0, 2.0), kwargs...)

Builds and simulates the scalar-controlled induction machine.

Example with stepped frequency:

```julia
sol, sys = simulate_scalar_im(
    tspan = (0.0, 10.0),
    frequency_profile = :steps,
    f_ref_val = 25.0,
    f_step_val = 5.0,
    t_hold_val = 1.0,
    t_step_val = 1.0,
    Tload_val = 0.0,
)
"""
function simulate_scalar_im(;
    tspan = (0.0, 2.0),
    frequency_profile = :constant,
    f_ref_val = 25.0,
    f_step_val = 5.0,
    t_hold_val = 1.0,
    t_step_val = 1.0,
    kwargs...
)

    sys = build_scalar_im_model(;
        frequency_profile = frequency_profile,
        f_ref_val = f_ref_val,
        f_step_val = f_step_val,
        t_hold_val = t_hold_val,
        t_step_val = t_step_val,
        kwargs...
    )

    prob = ODEProblem(sys, [], tspan)

    if frequency_profile == :steps
        n_steps = Int(ceil(f_ref_val / f_step_val))
        step_times = [t_hold_val + k * t_step_val for k in 0:n_steps]
        step_times = filter(τ -> tspan[1] < τ < tspan[2], step_times)

        sol = solve(
            prob,
            Rodas5P();
            reltol = 1e-6,
            abstol = 1e-8,
            tstops = step_times,
        )
    else
        sol = solve(
            prob,
            Rodas5P();
            reltol = 1e-6,
            abstol = 1e-8,
        )
    end

    return sol, sys
end

end