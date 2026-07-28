# Plan: refactor duplicated code out of `scripts/`

Goal: remove the copy-paste duplication between the 15 files in `scripts/`
without changing any numerical result. Every `results/*.csv` produced before the
refactoring must be byte-identical after it (modulo the one filename bug noted
below, which is fixed on purpose).

Status: **not started.** This document is analysis + plan only.

Scope: `scripts/` only. No changes to `src/`, `ext/`, or `test/`.

## Verified facts

State of the tree as of the `sysimage` branch, commit `bea185b`. Re-check if the
code moves under you.

- `scripts/` holds 15 files, 3466 lines: 12 `run_*.jl` scripts, `menu.jl`,
  `menu2.jl`, and `Project.toml`.
- **The package has an empty `[deps]`.** `Project.toml` declares only
  `[weakdeps]` `ModelingToolkit` / `OrdinaryDiffEq` and the
  `IM_AWES_benchMTKExt` extension. `CSV` and `DataFrames` are dependencies of
  `scripts/Project.toml`, **not** of the package.
  *Consequence:* none of the shared code proposed here can live in `src/`
  without adding hard dependencies and undoing the zero-dep design established
  in `oldplans/PlanPackageExtension.md`. It all belongs in a new
  `scripts/common.jl`.
- There is **no `src/profiles/`**. `CLAUDE.md` says there is; that is stale
  since the extension migration. The symbolic profile builders are in
  `ext/profiles/`, and the CSV playback interpolation helpers used by the
  hybrid simulators live inside `src/simulators/hybrid_foc_speed_f1_simulator.jl`
  (reused by `hybrid_foc_speed_mtpa_simulator.jl` — hence the include-order
  constraint).
- 11 of the 12 `run_*.jl` scripts write a CSV. `run_foc_speed_f1_160kw.jl` is
  the exception: it uses `Printf`/`Statistics` and prints only.
- 9 scripts define `case_name` and use `joinpath(results_dir, case_name * ".csv")`.
  `run_scalar_im.jl` and `run_scalar_frequency_steps.jl` instead hardcode
  `"scalar_im_25Hz_results.csv"` and `"scalar_frequency_steps_results.csv"`.
- **Bug found:** `run_foc_speed_f1_awes_profile_efficiency.jl:15` sets
  `case_name = "foc_speed_f1_awes_profile"` — identical to
  `run_foc_speed_f1_awes_profile.jl:15`. The two scripts overwrite each other's
  `results/foc_speed_f1_awes_profile.csv`. Direct fallout of the copy-paste.
  Phase 1 fixes this; it is the one intentional output change.

## The duplication, measured

### 1. `run_foc_speed_f1_awes_profile_efficiency.jl` is a near-verbatim copy

`diff run_foc_speed_f1_awes_profile.jl run_foc_speed_f1_awes_profile_efficiency.jl`
reports its first hunk at `353c353`: **lines 1-352 are identical**, including the
profile loading, all simulation settings, the `plotx` call, and the whole
`DataFrame`. The base script is 390 lines, the efficiency script 399.

The only real difference is the tail: a 8-line `trapz_integral` helper plus a
cycle-energy/efficiency summary that replaces the base script's "final values"
printout.

~350 duplicated lines to add ~30 lines of post-processing.

### 2. AWES profile loading — 3 identical copies

Lines 33-67 (~45 lines with comments) are identical in:

- `run_foc_speed_f1_awes_profile.jl`
- `run_foc_speed_f1_awes_profile_efficiency.jl`
- `run_foc_speed_mtpa_awes_profile.jl`

The block reads the CSV, shifts `profile_time` to start at 0, `sortperm`s by
time, drops duplicate timestamps via `[true; diff(...) .> 0.0]`, sets
`t_end = profile_time[end]`, and prints a "Profile check:" summary.

### 3. CSV export boilerplate — 11 scripts

Every one repeats the same four lines:

```julia
results_dir = joinpath(@__DIR__, "..", "results")
mkpath(results_dir)
csv_file = joinpath(results_dir, case_name * ".csv")
...
CSV.write(csv_file, df)
```

followed by `println("Saved simulation results to:")` / `println(csv_file)`.

Then each hand-writes a `DataFrame` of `collect(res.x)` calls — 23, 28, 35, 37,
51 and 59 columns respectively for the six hybrid FOC scripts. Comparing the
smallest (`run_foc_torque_f1_steps.jl`, 28 columns) with the largest
(`run_foc_speed_mtpa_awes_profile.jl`, 59), **26 columns are common**:

```
t_s  speed_rpm  torque_Nm  torque_obs_Nm  Te_ref_out_Nm  Tload_Nm
isd_ref_A  isq_ref_A  isd_obs_A  isq_obs_A  isd_ref_lim_A  isq_ref_lim_A
vsd_V  vsq_V  vs_alpha_V  vs_beta_V  is_alpha_A  is_beta_A
flux_r_Wb  theta_e_rad  omega_e_rad_s
voltage_saturation  sat_Te  sat_isd  sat_isq  vs_mod_unsat_V
```

The column *order* has already drifted: `Tload_Nm` / `TL_est_Nm` sit at
different positions in `run_foc_speed_f1_ramp_load_steps.jl` versus
`run_foc_speed_f1_ramp_load_estimator.jl`. This is what the shared writer
prevents.

### 4. Repeated constants

| Constant | Files |
|---|---|
| `Vs_max = 310.0` | 8 |
| `Is_max = 40.0` | 8 |
| `Ts = 100e-6` | 8 |
| `isd_nom = 23.04579328` | 6 |
| `use_filter/use_feedforward/use_saturation/use_antiwindup = true` | 7 |
| `plant_Rr_scale/obs_tau_r_scale/ctrl_k_scale = 1.0` | 6 |

A machine-rating change currently means editing eight files. Also
`.* 60 ./ (2π)` appears 14 times across the scripts.

### 5. Smaller items

- `run_foc_speed_f1_ramp_load_steps.jl` (271 lines) vs
  `run_foc_speed_f1_ramp_load_estimator.jl` (296 lines): differ only in the
  estimator settings, two `plotx` panels, and two summary lines.
- `menu.jl` and `menu2.jl` are the same ~30 lines of `RadioMenu` driver twice,
  differing only in the example list and the `2` suffix on `EXAMPLES`/
  `options`/`example_menu`.
- The 4-line `Pkg.activate` preamble is in all 15 files. **Leave it.** It has to
  run before `using IM_AWES_bench`, and it is what makes each script runnable
  standalone. Do not try to factor this out.

## Phases

Each phase is independently safe and independently verifiable. Do them in
order; stop and re-baseline after each.

### Phase 0 — baseline

1. Run all 11 CSV-producing scripts.
2. Copy `results/` to `results_baseline/` (add to `.gitignore` if not covered).
3. Note that the efficiency script currently clobbers the base script's CSV, so
   baseline them in a fixed order and keep both outputs under distinct names by
   hand.

Every later phase ends with: re-run, `diff` against the baseline, expect zero
differences.

### Phase 1 — `scripts/common.jl`

New file, included by the scripts that need it via
`include(joinpath(@__DIR__, "common.jl"))` after the `Pkg.activate` preamble.
It may use `CSV` and `DataFrames` because `scripts/Project.toml` has them.

Contents:

- `rad_s_to_rpm(x) = x .* 60 ./ (2π)` — replaces 14 inline conversions.
- `trapz_integral(t, y)` — lifted verbatim from the efficiency script.
- `load_awes_profile(path)` — the phase-2 block below, returning a named tuple
  `(time, speed_ref_rpm, load_torque_Nm, t_end)` plus the "Profile check:"
  printout (keep the printout: it is a real sanity check, and dropping it
  changes console output the user may rely on).
- `save_results(case_name, df)` — the `results_dir` / `mkpath` / `CSV.write` /
  `println` boilerplate. Returns the path.
- Default constants: `VS_MAX_DEFAULT = 310.0`, `IS_MAX_DEFAULT = 40.0`,
  `ISD_NOM_DEFAULT = 23.04579328`, `TS_DEFAULT = 100e-6`.

Do **not** hide the constants behind the simulators' own keyword defaults in
this phase — that is an `src/` change and belongs in a separate decision. Here
they stay visible in the scripts as `Vs_max = VS_MAX_DEFAULT`, so each script
still reads as a self-documenting case definition.

Then fix the `case_name` collision: set the efficiency script to
`"foc_speed_f1_awes_profile_efficiency"`.

Expected: -100 lines or so, plus the collision fix.

### Phase 2 — shared FOC results table

Add to `common.jl`:

```julia
foc_results_dataframe(res; extra...)
```

It builds the 26 common columns above from whatever fields `res` carries, in one
fixed order, and appends `extra` for the case-specific ones (`Te_ref_ext_Nm`,
`TL_est_Nm`, the MTPA diagnostics, the power columns, ...).

Guard each optional column with `hasproperty(res, :x)` rather than branching on
the simulator type — the result named tuples differ per simulator and this keeps
the writer additive.

Migrate the six hybrid FOC scripts one at a time, diffing the CSV after each.
**Column order must be preserved per script** or the diff will be noise; if a
script's historical order is inconsistent with the canonical order, that is a
deliberate output change — call it out rather than absorbing it silently.

Leave the three scalar/MTK scripts alone here. Their columns come from
`sol[sys.x]`, not from a hybrid result struct, and they share little with the
FOC set. A separate `scalar_results_dataframe` is possible but low value.

Expected: -350 to -400 lines.

### Phase 3 — collapse the efficiency script

With phases 1 and 2 done, `run_foc_speed_f1_awes_profile_efficiency.jl` should
be a short file. Two options:

1. **Preferred:** add a `compute_efficiency = false` flag near the top of
   `run_foc_speed_f1_awes_profile.jl`, and have the efficiency script set the
   flag, set its own `case_name`, and `include` the base script. Keeps one
   simulation definition.
2. If flag-and-include reads badly in a menu-driven workflow, keep two scripts
   but have both call one `run_awes_speed_case(; ...)` function defined in
   `common.jl`.

Pick one when you get there; do not build both.

### Phase 4 — unify the menus

Move the `RadioMenu` driver into `common.jl`:

```julia
example_menu(examples; pagesize = 9)
```

`menu.jl` and `menu2.jl` then reduce to their `EXAMPLES` list plus one call.
Keep the two separate entry points and keep their header comments — the split
between hybrid and MTK examples is deliberate (menu2 pays for loading
ModelingToolkit and OrdinaryDiffEq) and must survive the refactoring.

Preserve the absolute-path `Base.include(Main, joinpath(@__DIR__, ...))` call
and its comment verbatim. That is a real fix for relative-include resolution,
not incidental.

### Phase 5 (optional, decide separately) — the two ramp-load scripts

Merging `run_foc_speed_f1_ramp_load_steps.jl` and
`run_foc_speed_f1_ramp_load_estimator.jl` behind a `load_estimator` switch would
save another ~250 lines, but it costs the "one script = one readable case
definition" property that the rest of `scripts/` has, and their `plotx` panels
genuinely differ. **Recommendation: do not do this** unless the two drift
further apart in maintenance. Listed here so the option is recorded, not
forgotten.

## Explicitly out of scope

- Anything under `src/` or `ext/`. In particular, do not move
  `load_awes_profile` into the package — that would add `CSV` and `DataFrames`
  to a deliberately dependency-free package.
- The `plotx` calls. They look repetitive but the panel/label/ylabel structure
  is genuinely per-case, and a generic plotting wrapper would take more
  arguments than the call it replaces.
- The `Pkg.activate` preamble (see item 5 above).
- Changing any simulator keyword defaults.

## Expected outcome

Phases 1-4 should take `scripts/` from ~3466 lines to roughly ~2000, with no
change to any CSV output except the corrected efficiency-script filename.

## Update `CLAUDE.md` when done

Two things there go stale under this plan:

- The `src/profiles/` bullet in the "Layout" section is already wrong (see
  Verified facts) — fix it while you are in there.
- The scripts list under "Commands" does not mention `common.jl`; add a note
  that scripts share helpers from it.
