using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Test
using IM_AWES_bench

# ============================================================
# Subprocess helper
# ============================================================
#
# Extension loading is one-way within a session: once the trigger packages are
# loaded the extension stays loaded, so "absent before, present after" cannot be
# tested with two testsets in one process. Each probe runs in a fresh julia.

"""
    probe(code) -> (; ok, out, err)

Run `code` in a fresh julia process on the test environment. Returns the
process's stdout, stderr, and whether it exited cleanly.
"""
function probe(code::AbstractString)
    out = IOBuffer()
    err = IOBuffer()
    cmd = `$(Base.julia_cmd()) --startup-file=no --project=$(@__DIR__) -e $code`
    p = run(pipeline(ignorestatus(cmd); stdout = out, stderr = err))
    return (; ok = p.exitcode == 0, out = String(take!(out)), err = String(take!(err)))
end

# Printed by every probe below. Method count is the right signal rather than
# `isdefined`: the stub declarations in the main module mean the builder names
# always exist, and only gain a method once the extension loads.
const PROBE_PRELUDE = """
    loaded(name) = any(k -> k.name == name, keys(Base.loaded_modules))
    nmeth(f) = length(methods(f))
    report(tag) = println(
        tag, " mtk=", loaded("ModelingToolkit"), " ode=", loaded("OrdinaryDiffEq"),
        " scalar=", nmeth(IM_AWES_bench.build_scalar_im_system),
        " foc=", nmeth(IM_AWES_bench.build_foc_current_im_system),
    )
"""

function parse_report(out, tag)
    line = only(filter(l -> startswith(l, tag), split(strip(out), '\n')))
    fields = Dict(split(kv, '=')[1] => split(kv, '=')[2] for kv in split(line)[2:end])
    return (;
        mtk = parse(Bool, fields["mtk"]),
        ode = parse(Bool, fields["ode"]),
        scalar = parse(Int, fields["scalar"]),
        foc = parse(Int, fields["foc"]),
    )
end

@testset "IM_AWES_bench" begin

    # ========================================================
    # Hybrid simulators: the MTK-free fast path
    # ========================================================

    @testset "hybrid simulators run" begin
        t_end = 0.05
        Ts = 100e-6
        expected = round(Int, t_end / Ts)

        sims = [
            "current" => () -> simulate_foc_current_hybrid(; t_end, Ts),
            "torque_f1" => () -> simulate_foc_torque_f1_hybrid(; t_end, Ts),
            "speed_f1" => () -> simulate_foc_speed_f1_hybrid(; t_end, Ts),
            "speed_mtpa" => () -> simulate_foc_speed_mtpa_hybrid(; t_end, Ts),
        ]

        for (name, run_sim) in sims
            @testset "$name" begin
                res = run_sim()
                @test length(res.t) >= expected
                @test res.t[1] == 0.0
                @test res.t[end] ≈ t_end atol = 2Ts
                @test issorted(res.t)
                @test all(isfinite, res.t)
                @test all(isfinite, res.speed_rpm)
                @test all(isfinite, res.torque)
                @test length(res.speed_rpm) == length(res.t)
                @test length(res.torque) == length(res.t)
            end
        end
    end

    # ========================================================
    # Extension loading
    # ========================================================
    #
    # The whole point of the MTK extension: a bare `using IM_AWES_bench` must
    # stay free of the symbolic and solver stacks. If these fail, the package
    # has quietly reacquired a hard dependency on ModelingToolkit.

    @testset "bare load pulls in no symbolic stack" begin
        r = probe(PROBE_PRELUDE * """
            using IM_AWES_bench
            report("BARE")
        """)
        @test r.ok
        bare = parse_report(r.out, "BARE")
        @test !bare.mtk
        @test !bare.ode
        @test bare.scalar == 0
        @test bare.foc == 0
    end

    # Julia loads an extension only once *every* package in its trigger list is
    # present. ModelingToolkit alone is not enough here, because the simulate_*
    # functions default to Rodas5P() and call solve, so OrdinaryDiffEq cannot be
    # dropped from the trigger list. This pins the user-facing contract.
    @testset "ModelingToolkit alone does not trigger the extension" begin
        r = probe(PROBE_PRELUDE * """
            using IM_AWES_bench
            using ModelingToolkit
            report("MTKONLY")
        """)
        @test r.ok
        mtkonly = parse_report(r.out, "MTKONLY")
        @test mtkonly.mtk
        @test mtkonly.scalar == 0
        @test mtkonly.foc == 0
    end

    @testset "both triggers load the extension" begin
        r = probe(PROBE_PRELUDE * """
            using IM_AWES_bench
            using ModelingToolkit
            using OrdinaryDiffEq
            report("WITHMTK")
        """)
        @test r.ok
        withmtk = parse_report(r.out, "WITHMTK")
        @test withmtk.mtk
        @test withmtk.ode
        @test withmtk.scalar >= 1
        @test withmtk.foc >= 1
    end

    # The alias is a `const` in the main module pointing at the stub, because an
    # extension cannot create a binding in its parent. It must still resolve to
    # the same function object once the extension has added its method.
    @testset "build_scalar_im_model alias" begin
        r = probe("""
            using IM_AWES_bench, ModelingToolkit, OrdinaryDiffEq
            println("ALIAS ", build_scalar_im_model === build_scalar_im_system,
                    " ", length(methods(build_scalar_im_model)))
        """)
        @test r.ok
        @test occursin("ALIAS true 1", r.out)
    end

    # ========================================================
    # MTK path
    # ========================================================
    #
    # Short runs, but they exercise build -> structural_simplify -> solve, which
    # is the only coverage that would catch a scoping mistake in an equation
    # builder inside the extension (e.g. `D` no longer in scope).

    @testset "MTK path" begin
        @eval using ModelingToolkit
        @eval using OrdinaryDiffEq

        @testset "simulate_scalar_im" begin
            tspan = (0.0, 0.05)
            sol, sys = simulate_scalar_im(; tspan)
            @test sys !== nothing
            @test length(sol.t) > 1
            @test sol.t[1] == tspan[1]
            @test sol.t[end] ≈ tspan[2]
            @test all(u -> all(isfinite, u), sol.u)
        end

        # The observer branch of the scalar system once threw
        # `FieldError: type NamedTuple has no field wsl_obs` — the slip
        # equations were added to build_rotor_flux_observer_eqs for the FOC
        # system and never back-ported here. Nothing exercised it, so it rotted.
        @testset "simulate_scalar_im with observer" begin
            sol, sys = simulate_scalar_im(; tspan = (0.0, 2.0), include_observer = true)
            @test length(sol.t) > 1
            @test all(u -> all(isfinite, u), sol.u)
            @test isfinite(sol(2.0, idxs = sys.ws_obs))
            @test isfinite(sol(2.0, idxs = sys.wsl_obs))

            # The observer must reconstruct the plant torque, not merely run.
            # They agree to ~1e-4 N m once the startup transient has passed.
            for t in (1.0, 1.5, 2.0)
                @test sol(t, idxs = sys.Te_obs) ≈ sol(t, idxs = sys.Te) atol = 1e-3
            end
            # Rotor flux settles near its rated value.
            @test sol(2.0, idxs = sys.flux_r_mod_obs) ≈ 0.893 atol = 0.02
        end

        @testset "simulate_foc_current_im" begin
            tspan = (0.0, 0.05)
            sol, sys = simulate_foc_current_im(; tspan)
            @test sys !== nothing
            @test length(sol.t) > 1
            @test sol.t[1] == tspan[1]
            @test sol.t[end] ≈ tspan[2]
            @test all(u -> all(isfinite, u), sol.u)
        end

        # Regression guard for the zero-flux NaN in the analytical Jacobian.
        # sqrt(λrα^2 + λrβ^2) without the epsilon differentiates to 0/0 at the
        # zero-flux initial condition, which is invisible to a finite-difference
        # Jacobian and only shows up through MTK's symbolic one.
        @testset "analytical Jacobian is finite at zero flux" begin
            sys = build_foc_current_im_system()
            prob = ODEProblem(sys, [], (0.0, 0.05); jac = true)
            n = length(prob.u0)
            J = zeros(n, n)
            prob.f.jac(J, prob.u0, prob.p, 0.0)
            @test !any(isnan, J)
            @test !any(isinf, J)
        end

        # The controller must actually track its references, not merely produce
        # finite numbers. isd steps to 10 A at t = 1, isq to +15/-15/0 A.
        @testset "FOC current tracking" begin
            sol, sys = simulate_foc_current_im(; tspan = (0.0, 12.0))
            for (t, isd_ref, isq_ref) in
                [(2.5, 10.0, 0.0), (5.0, 10.0, 15.0), (8.0, 10.0, -15.0), (11.0, 10.0, 0.0)]
                @test sol(t, idxs = sys.isd_e_obs) ≈ isd_ref atol = 0.1
                @test sol(t, idxs = sys.isq_e_obs) ≈ isq_ref atol = 0.1
            end
            # Rotor flux magnitude settles at Lm * isd.
            @test sol(11.0, idxs = sys.flux_r_mod_obs) ≈ 0.04084 * 10 atol = 0.01
        end
    end
end
