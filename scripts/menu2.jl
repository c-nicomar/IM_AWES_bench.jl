using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using REPL.TerminalMenus: RadioMenu, request

# Menu of the ModelingToolkit examples, the counterpart of menu.jl. Every script
# listed here calls one of the symbolic system builders, which live in the
# IM_AWES_benchMTKExt package extension, so the first run pays for loading
# ModelingToolkit and OrdinaryDiffEq.
const EXAMPLES = [
    "run_scalar_im.jl",                          # scalar V/f, constant frequency
    "run_scalar_frequency_steps.jl",             # scalar V/f, frequency steps
    "run_scalar_frequency_steps_load_steps.jl",  # ... plus load steps and observer
    "run_foc_current_steps.jl",                  # symbolic FOC inner current loop
]

options = [chopsuffix(file, ".jl") * " = include(\"" * file * "\")"
           for file in EXAMPLES]
push!(options, "quit")

function example_menu2()
    active = true
    while active
        menu = RadioMenu(options, pagesize=9)
        choice = request("\nChoose example to run or `q` to quit: ", menu)

        if choice != -1 && choice != length(options)
            # A relative `include` only resolves against this file while
            # menu2.jl itself is being included, so use the absolute path; that
            # keeps `example_menu2()` working when it is called again later.
            Base.include(Main, joinpath(@__DIR__, EXAMPLES[choice]))
        else
            println("Left menu. Press <ctrl><d> to quit Julia!")
            active = false
        end
    end
end

example_menu2()
