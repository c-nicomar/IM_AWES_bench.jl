module IM_AWES_bench_jl

using ModelingToolkit
using ModelingToolkit: t_nounits as t, D_nounits as D
using OrdinaryDiffEq

export build_scalar_im_model, simulate_scalar_im

"""
    build_scalar_im_model(; kwargs...)

Builds a simplified squirrel-cage induction machine model driven by open-loop scalar V/f control.

This first version uses stationary αβ equations, not full Simscape-style electrical pins.
"""
function build_scalar_im_model(;
    Rs_val = 0.21946,
    Rr_val = 0.11659,
    Lls_val = 4.28e-3,
    Llr_val = 4.28e-3,
    Lm_val = 40.84e-3,
    p_val = 2.0,
    J_val = 0.2685 + 0.1,
    B_val = 0.01298,
    Vll_nom_val = 380.0,
    f_nom_val = 50.0,
    f_ref_val = 25.0,
    Tload_val = 0.0,
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
        θs(t) = 0.0
        isα(t) = 0.0
        isβ(t) = 0.0
        irα(t) = 0.0
        irβ(t) = 0.0
        ωm(t) = 0.0
        θm(t) = 0.0

        ψsα(t)
        ψsβ(t)
        ψrα(t)
        ψrβ(t)

        va(t)
        vb(t)
        vc(t)
        vsα(t)
        vsβ(t)

        Te(t)
        n_rpm(t)
    end

    Ls = Lls + Lm
    Lr = Llr + Lm

    Vphase_peak = sqrt(2) * Vll_nom / sqrt(3) * f_ref / f_nom

    eqs = [
        # Scalar V/f control
        D(θs) ~ 2π * f_ref,

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
        J * D(ωm) ~ Te - Tload - B * ωm,
        D(θm) ~ ωm,

        # Useful output
        n_rpm ~ ωm * 60 / (2π),
    ]

    @named sys = ODESystem(eqs, t)
    return structural_simplify(sys)
end


"""
    simulate_scalar_im(; tspan=(0.0, 2.0), kwargs...)

Builds and simulates the scalar-controlled induction machine.
"""
function simulate_scalar_im(; tspan = (0.0, 2.0), kwargs...)
    sys = build_scalar_im_model(; kwargs...)
    prob = ODEProblem(sys, [], tspan)
    sol = solve(prob, Rodas5P(); reltol = 1e-6, abstol = 1e-8)
    return sol, sys
end

end