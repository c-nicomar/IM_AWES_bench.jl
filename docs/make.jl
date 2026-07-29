using Documenter
using IM_AWES_bench

# Both extension triggers, so IM_AWES_benchMTKExt loads and the docstrings that
# live in ext/ become visible to Documenter. OrdinaryDiffEq is not optional:
# Julia loads an extension only once every package in its trigger list is
# present, and the simulate_* functions default to Rodas5P() and call solve.
using ModelingToolkit, OrdinaryDiffEq

const MTKExt = Base.get_extension(IM_AWES_bench, :IM_AWES_benchMTKExt)
MTKExt === nothing && error(
    "IM_AWES_benchMTKExt did not load. Both ModelingToolkit and OrdinaryDiffEq " *
    "must be present, or half the public surface documents as zero-method stubs."
)

DocMeta.setdocmeta!(IM_AWES_bench, :DocTestSetup, :(using IM_AWES_bench); recursive = true)

makedocs(;
    modules  = [IM_AWES_bench, MTKExt],
    authors  = "Carolina Nicolás <canicola@ing.uc3m.es> and contributors",
    sitename = "IM_AWES_bench.jl",
    format = Documenter.HTML(;
        canonical  = "https://c-nicomar.github.io/IM_AWES_bench.jl",
        edit_link  = "main",
        # Clean URLs in CI; file:// friendly locally.
        prettyurls = get(ENV, "CI", "false") == "true",
        assets     = String[],
    ),
    pages = [
        "Home"               => "index.md",
        "Package overview"   => "overview.md",
        "Architecture"       => "architecture.md",
        "MTPA integration"   => "mtpa.md",
        "160 kW FOC F1 case" => "foc_f1_160kw.md",
        "Developer guide"    => "developer_guide.md",
        "API" => [
            "Package"       => "api/package.md",
            "Controllers"   => "api/controllers.md",
            "Estimators"    => "api/estimators.md",
            "Simulators"    => "api/simulators.md",
            "MTK extension" => "api/mtk.md",
        ],
    ],
    # Every source file in src/ and ext/ is covered by an @autodocs block on one
    # of the API pages, so any docstring that exists is published. That makes it
    # safe to run strict: an unpublished docstring now fails the build.
    #
    # Note what this does NOT do: checkdocs compares docstrings that exist
    # against docstrings spliced into the manual. An exported name with no
    # docstring at all is invisible to it -- 29 of 36 exports are in that state
    # today. Coverage is tracked in Step 5 of PlanDocu.md, not by this setting.
    checkdocs = :exports,
)

deploydocs(;
    repo      = "github.com/c-nicomar/IM_AWES_bench.jl",
    devbranch = "main",
    push_preview = true,
)
