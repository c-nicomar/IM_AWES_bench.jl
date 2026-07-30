# Developer guide

This page documents the developer tools and scripts in the `bin/` directory, and
explains how to run simulations, build documentation, and work with the package
effectively.

## Prerequisites

- **Julia 1.12** — required by the project. Install it with
  [juliaup](https://github.com/JuliaLang/juliaup):
  ```bash
  juliaup add 1.12 && juliaup default 1.12
  ```

## Scripts in `bin/`

All scripts are designed to be run from the repository root (or from the `bin/`
directory itself — they auto-detect and `cd ..` when needed).

### `bin/install`

Set up the workspace environment. Run this once when you clone the repository.

It performs the following steps:

1. **Checks the Julia version** — verifies that `julia` resolves to Julia 1.12
   and exits with a clear error otherwise.
2. **Restores the known-good manifest** — copies
   `Manifest-v1.12.toml.default` to `Manifest-v1.12.toml`, so every developer
   starts with the exact same dependency resolution.
3. **Removes stale sub-project manifests** — deletes any `Manifest.toml` inside
   `scripts/` and `test/` that would shadow the workspace-level manifest and
   break resolution.
4. **Instantiates the workspace** — runs `Pkg.instantiate()` on the root
   project, downloading and precompiling all dependencies.
5. **Precompiles the scripts project** — runs `Pkg.precompile()` on
   `scripts/Project.toml` so simulation scripts start faster.

```bash
bin/install
```

### `bin/run_julia`

Start a Julia REPL configured for the workspace. This is the primary way to
interact with the package interactively.

Features:

- Launches Julia with `-t auto` (all available threads) and `--project=.`
- Defines two convenience functions on startup:
  - `menu()` — lists and runs the hybrid FOC simulation scripts
  - `menu2()` — lists and runs the ModelingToolkit-based scripts
- Loads a custom system image (`bin/sysimage.so` / `.dylib` / `.dll`) if one
  exists, which skips precompilation of heavy dependencies like `ModelingToolkit`
  and `MakieControlPlots` for faster startup.
- Sets `KMP_DUPLICATE_LIB_OK=TRUE` to avoid OpenMP conflicts on some platforms.

```bash
bin/run_julia                     # normal startup
bin/run_julia --nosysimage        # ignore custom system image
bin/run_julia -e 'include(...)'   # run a script non-interactively
```

### `bin/create_sys_image`

Build a custom system image that bakes in `ModelingToolkit`, `OrdinaryDiffEq`,
`MakieControlPlots`, `CSV`, and `DataFrames`. This makes `bin/run_julia` start
much faster because these heavy dependencies are already compiled into the
binary.

**Important**: `InductionMachineDrives` itself is deliberately **not** baked into the
system image, so its source stays editable under Revise without requiring a
rebuild.

```bash
bin/create_sys_image    # takes 10–60 minutes
```

The resulting image is written to `bin/sysimage.so` (or `.dylib` on macOS,
`.dll` on Windows), which is gitignored. Rebuild it after dependency version
changes; delete the file (or use `--nosysimage`) to fall back to the default
system image.

### `bin/build_docs`

Build the Documenter.jl documentation into `docs/build/`.

It first instantiates the `docs/` environment (which resolves independently from
the workspace — it has its own `Project.toml`) and then runs `docs/make.jl`.

```bash
bin/build_docs
```

Open the result at `docs/build/index.html`. Note that `ModelingToolkit` and
`OrdinaryDiffEq` are loaded during the build so that docstrings from the MTK
extension are visible, but the *default* system image is used (never `-J`), to
avoid version mismatches with the docs environment.

### `bin/serve_docs`

Serve the documentation locally with live reload at `http://localhost:8000`,
opening the browser automatically.

```bash
bin/serve_docs              # open browser automatically
bin/serve_docs --no-browser # skip browser launch (e.g. over SSH)
```

Internally this calls `Documenter.servedocs(launch_browser = true)`, which
rebuilds whenever a file in `docs/src/` changes. To also pick up docstring
edits from `src/` and `ext/`, edit the script to add `include_dirs = ["src",
"ext"]` to the `servedocs(...)` call.

Requires `LiveServer`, which is auto-installed into the global environment if
absent.

### `bin/jetls`

Run [JET.jl](https://github.com/aviatesk/JET.jl) static analysis on the main
package source.

```bash
bin/jetls
```

This checks `src/` for type-inference errors, method redefinitions, and similar
problems. JET must be added to the environment first; it is deliberately kept
out of the project dependencies to keep the dependency tree small.

## Running simulations

The simulation scripts live in `scripts/`. Each script activates the common
`scripts/Project.toml` environment. You can run them from the command line:

```bash
julia scripts/run_foc_speed_f1_ramp_load_steps.jl
```

Or interactively from the REPL started by `bin/run_julia` using `menu()` or
`menu2()`.

Results (CSV files and occasionally JLD2 snapshots) are written to the
`results/` directory, which is gitignored.

## Running tests

Run the full test suite from the command line:

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Or in a REPL:

```julia
include("test/runtests.jl")
```

The test suite is intentionally slow — four of the testsets spawn fresh Julia
subprocesses to verify extension-loading contracts, and the MTK testsets pay
for `mtkcompile` plus extension precompilation. Progress is printed with
timestamps so a healthy run is distinguishable from a hang.

## Documentation conventions

- New `.md` pages go into `docs/src/`.
- To add a page to the navigation sidebar, edit the `pages` array in
  `docs/make.jl`.
- API docstrings are auto-generated by `Documenter` from the source code via
  `@autodocs` blocks in `docs/src/api/*.md`. Every function, type, and constant
  in `src/` and `ext/` is covered, so any docstring that exists is published.