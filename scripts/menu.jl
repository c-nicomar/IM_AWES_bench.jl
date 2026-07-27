using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using REPL.TerminalMenus: RadioMenu, request

# Menu of the hybrid FOC examples. Only scripts that run on the hand-written
# plant/controller path are listed; the ModelingToolkit-based scripts are left
# out on purpose, because they pull in the MTK/OrdinaryDiffEq extension. They
# live in menu2.jl.
const EXAMPLES = [
    "run_foc_current_hybrid_steps.jl",       # inner current loop, id/iq steps
    "run_foc_torque_f1_steps.jl",            # outer torque loop, constant flux
    "run_foc_speed_f1_ramp_load_steps.jl",   # outer speed loop, load steps
    "run_foc_speed_f1_ramp_load_estimator.jl",  # ... plus Kalman load estimator
    "run_foc_speed_f1_awes_profile.jl",      # speed loop on AWES CSV profile
    "run_foc_speed_f1_awes_profile_efficiency.jl",  # ... plus efficiency plots
    "run_foc_speed_mtpa_awes_profile.jl",    # MTPA speed loop on AWES profile
    "run_foc_speed_f1_160kw.jl",             # 160 kW machine, limits and power
]

options = [chopsuffix(file, ".jl") * " = include(\"" * file * "\")"
           for file in EXAMPLES]
push!(options, "quit")

function example_menu()
    active = true
    while active
        menu = RadioMenu(options, pagesize=9)
        choice = request("\nChoose example to run or `q` to quit: ", menu)

        if choice != -1 && choice != length(options)
            # A relative `include` only resolves against this file while
            # menu.jl itself is being included, so use the absolute path; that
            # keeps `example_menu()` working when it is called again later.
            Base.include(Main, joinpath(@__DIR__, EXAMPLES[choice]))
        else
            println("Left menu. Press <ctrl><d> to quit Julia!")
            active = false
        end
    end
end

example_menu()
