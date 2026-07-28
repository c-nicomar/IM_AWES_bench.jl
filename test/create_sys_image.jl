# Build a custom system image containing the large dependencies of this
# project (ModelingToolkit, OrdinaryDiffEq, Makie/MakieControlPlots, CSV,
# DataFrames). The image lands in `bin/` and is picked up automatically by
# `bin/run_julia`.
#
# Usage:
#     bin/create_sys_image
# or
#     julia -t auto test/create_sys_image.jl
#
# IM_AWES_bench itself is deliberately *not* baked into the image: it is the
# package under development and must stay editable (Revise) without a rebuild.

using Pkg

const ROOT    = dirname(@__DIR__)
const SCRIPTS = joinpath(ROOT, "scripts")
const BIN     = joinpath(ROOT, "bin")

# The scripts environment is the superset of everything used interactively,
# so the image is built against it.
const PACKAGES = [
    "ModelingToolkit",
    "OrdinaryDiffEq",
    "MakieControlPlots",
    "CSV",
    "DataFrames",
]

using Libdl: dlext
const SYSIMAGE_PATH = joinpath(BIN, "sysimage." * dlext)

# ------------------------------------------------------------
# Precompilation workload
# ------------------------------------------------------------
# Executed while tracing, so that the compiled code of a representative
# simulation ends up in the image. Every step is guarded: a workload that
# fails should degrade the image, not abort the build.
const WORKLOAD = """
using IM_AWES_bench
using ModelingToolkit
using OrdinaryDiffEq
using MakieControlPlots
using CSV
using DataFrames

try
    # Hybrid FOC path: discrete controllers + hand-written plant stepper.
    simulate_foc_current_hybrid(t_end = 0.05, Ts = 100e-6)
catch err
    @warn "sysimage workload: hybrid FOC run failed" err
end

try
    # Symbolic path: exercises MTK model building and the ODE solver.
    simulate_scalar_im(tspan = (0.0, 0.05), f_ref_val = 25.0, Tload_val = 0.0)
catch err
    @warn "sysimage workload: scalar MTK run failed" err
end

try
    io = IOBuffer()
    CSV.write(io, DataFrame(t = [0.0, 1.0], y = [1.0, 2.0]))
    CSV.read(IOBuffer(String(take!(io))), DataFrame)
catch err
    @warn "sysimage workload: CSV/DataFrames round-trip failed" err
end
"""

# ------------------------------------------------------------
# Build
# ------------------------------------------------------------

@info "Instantiating $SCRIPTS"
Pkg.activate(SCRIPTS)
Pkg.instantiate()

# PackageCompiler lives in a throw-away environment so it does not become a
# dependency of the project itself.
Pkg.activate(; temp = true)
Pkg.add("PackageCompiler")
using PackageCompiler

workload_file = joinpath(tempdir(), "im_awes_bench_sysimage_workload.jl")
write(workload_file, WORKLOAD)

mkpath(BIN)

@info "Creating system image" packages = PACKAGES path = SYSIMAGE_PATH
create_sysimage(
    PACKAGES;
    sysimage_path = SYSIMAGE_PATH,
    project = SCRIPTS,
    precompile_execution_file = workload_file,
    incremental = true,
)

rm(workload_file; force = true)

@info """
System image written to $SYSIMAGE_PATH
`bin/run_julia` picks it up automatically. Rebuild it after changing
dependency versions; delete the file to fall back to the default image."""
