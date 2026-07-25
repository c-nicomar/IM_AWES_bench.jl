# Replace ControlPlots with MakieControlPlots

ControlPlots depends on Python/matplotlib. MakieControlPlots reimplements the same API on top of
Makie, which removes that dependency and makes the installation easier.

## Scope

ControlPlots is used in **both** `examples/` and `scripts/` — 11 run scripts and 2 `Project.toml`
files in total:

| Location | Files with `using ControlPlots` |
|---|---|
| `examples/` | 1 (`run_scalar_im.jl`) |
| `scripts/` | 10 |

Both must be migrated. Migrating only `examples/` leaves ControlPlots in the `scripts/` project, so
Python/matplotlib would still be required and the benefit would not be achieved.

ControlPlots is the only Python-dependent package in this repository, so completing the migration
removes the Python dependency entirely.

## Feasibility

The plotting surface used here is small: only `plotx` (12 call sites) and `display` (11 call sites).
No use of `plot`, `plotxy`, `plot2d` or `savefig`. MakieControlPlots provides both `plotx` and
`Base.display(::PlotX)`, so the `p = plotx(...); display(p)` idiom carries over unchanged.

MakieControlPlots is registered in the General registry (v0.1.9) and declares `julia = "1.11, 1.12"`,
compatible with this repository's `julia = "1.12"`.

## Steps

1. `examples/Project.toml` and `scripts/Project.toml`: replace the `ControlPlots` dependency with
   `MakieControlPlots`.
2. All 11 run scripts: `using ControlPlots` → `using MakieControlPlots`.
3. All 12 `plotx` calls: rename the two keyword arguments that differ. MakieControlPlots' `plotx`
   has an explicit keyword list with no `kwargs...` catch-all, so the ControlPlots names are **not**
   silently ignored — they raise a `MethodError`:

   ```julia
   legend_size = 9,   # → legendsize = 9
   loc = "best",      # → drop it (MakieControlPlots defaults to legend_position = :auto)
   ```

   Note that `loc` takes a matplotlib location string, whereas `legend_position` takes a corner
   symbol (`:auto`, `:rt`, …); `:auto` performs the automatic best-corner search, so it is the
   faithful translation of `"best"`.
4. `docs/README_IM_AWES_bench_jl.md`: delete the "OpenMP/plotting error on Windows" section. The
   `ENV["KMP_DUPLICATE_LIB_OK"] = "TRUE"` workaround it documents exists only because of matplotlib.
5. Verify: run each script, confirm the plots render as before.

## Trade-off to be aware of

The Python/matplotlib dependency is replaced by hard dependencies on **both GLMakie and CairoMakie**.
Installation becomes simpler, but not lighter, and GLMakie requires working OpenGL. MakieControlPlots
documents a CairoMakie fallback for headless use; since `display()` is called unconditionally in all
11 scripts, this fallback should be confirmed on a headless machine before relying on it there.
First-plot latency shifts from Python startup to Makie precompilation.
