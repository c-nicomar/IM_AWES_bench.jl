# Plan: move MTK-dependent code into a package extension

Goal: `using IM_AWES_bench` loads with no symbolic/solver stack. The hybrid FOC
simulators are the fast path and must not pay for ModelingToolkit. The
scalar/FOC-current MTK builders become available when the user also loads
ModelingToolkit.

Status: phases 1-4 done. Migration complete. Julia 1.12, extensions available.

## Verified facts

Established by inspecting the tree; re-check if the code moves under you.

- The four `simulate_*_hybrid` functions in `src/simulators/` contain no MTK
  tokens and call none of the `build_*_eqs`, `build_*_system`, or `*_tstops`
  helpers. The hybrid path and the MTK path are disjoint.
- `src/IM_AWES_bench.jl:6-7` is the only `using ModelingToolkit` in the package.
  The `D_nounits as D` alias from line 7 is what puts `D` in scope for every
  equation builder, so those builders are coupled to MTK even though their
  bodies never name it.
- Only four functions construct or solve systems: `build_scalar_im_system`,
  `simulate_scalar_im`, `build_foc_current_im_system`, `simulate_foc_current_im`.
- Six equation builders return `Vector{Equation}` built with `~` and `D(...)`:
  `build_scalar_vf_control_eqs`, `build_foc_current_controller_eqs`,
  `build_rotor_flux_observer_eqs`, `build_induction_machine_alpha_beta_eqs`,
  `build_frequency_command_eqs`, `build_load_torque_eqs`.
- Every call site of those builders and of `frequency_profile_tstops` /
  `load_profile_tstops` is inside `src/systems/`. None are exported.
- `solve` appears only in the two `src/systems/` files.
- `ModelingToolkitStandardLibrary`, `DifferentialEquations`, `MAT`,
  `DataInterpolations`, `CSV`, and `DataFrames` are declared in `Project.toml`
  and referenced nowhere in `src/`.
- ~~`test/runtests.jl` asserts `1+1 == 2` and nothing else.~~ Replaced in
  phase 2; see below.
- The manifest is `Manifest-v1.12.toml` — Julia's version-specific manifest
  name requires a **hyphen**; `Manifest_v1.12.toml` with an underscore is not a
  name Julia recognizes (`Base.manifest_names`) and leaves the project with no
  manifest at all. It is gitignored (`.gitignore:9,32-33`), not tracked — so dependency
  changes leave no diff to review and every clone resolves fresh.

## Phase 1 — drop dead dependencies

Independent of the extension work and worth doing first: it is reversible and
needs no API redesign. Done — but see the note under Load time: the expectation
that this would cut load time was wrong, so the payoff here is a smaller install
and resolve footprint, not a faster `using`.

- [x] Remove from `[deps]` in `Project.toml`: `DifferentialEquations`, `MAT`,
      `DataInterpolations`, `CSV`, `DataFrames`, `ModelingToolkitStandardLibrary`.
      Root project is now `ModelingToolkit` + `OrdinaryDiffEq` only.
- [x] Leave `CSV`/`DataFrames` in `scripts/Project.toml` — the scripts do use them.
      `examples/Project.toml` already declared them too.
- [x] `Pkg.resolve()`, then confirm `using IM_AWES_bench` still works. The four
      unused packages left `Manifest.toml` entirely; `CSV`/`DataFrames` remain,
      correctly, as workspace members still require them.
- [x] Confirm the workspace still resolves: `using CSV, DataFrames, IM_AWES_bench`
      succeeds under both `--project=scripts` and `--project=examples`.
      `test/runtests.jl` passes. Note that no script was actually *run* — the
      plotting scripts open a window, so this checks resolution, not execution.
- [x] Record `@time using IM_AWES_bench` before and after, in the table below.

## Phase 2 — test scaffolding

Do this before moving files, so the move has something to fail against.

- [x] In `test/runtests.jl`, add a testset that loads `IM_AWES_bench` alone and
      asserts the four hybrid simulators are callable. All four run at
      `t_end = 0.05`, `Ts = 100e-6`, checked for sample count, monotonic time,
      finite `speed_rpm`/`torque`, and matching array lengths.
- [x] Add a testset asserting the MTK builders are *not* yet available, then
      `using ModelingToolkit`, then asserting they are. Implemented via a
      `probe()` helper that runs each check in a fresh `julia -e` subprocess.
      The signal is *method count*, not `isdefined` — after phase 3 the stub
      declarations mean the names exist with zero methods until MTK loads.
      Gated on `EXT_MIGRATED`, auto-detected from `[extensions]` in the root
      `Project.toml`, so the assertions turn real on their own after phase 3
      with no constant to flip. Currently 3 `@test_broken`.
- [x] Add a smoke test: one short `simulate_scalar_im` run and one
      `simulate_foc_current_im` run. `simulate_scalar_im` passes.
      `simulate_foc_current_im` does **not** — see below.
- [x] Add `ModelingToolkit` and `TOML` to `test/Project.toml`.

Result: 62 pass, 3 broken, 0 fail, ~39 s. The 3 remaining broken are the
intentional phase-3 load-isolation tripwires.

### Found while writing the tests: `simulate_foc_current_im` was broken — FIXED

Pre-existing, unrelated to this migration. The solve aborted at `t = 0` with
retcode `Unstable` — "dt was forced below floating point epsilon" — under both
`Rodas5P` and `FBDF`, at every `tspan` tried. `scripts/run_foc_current_steps.jl`
was broken by the same cause.

Root cause: **NaN in MTK's analytical Jacobian at the zero-flux initial
condition.** `sqrt(x)` has an infinite derivative at `x = 0`, so

    flux_r_mod_obs ~ sqrt(λrα_obs^2 + λrβ_obs^2)

differentiates to `λrα / sqrt(λrα^2 + λrβ^2)` = `0/0` = NaN when the machine
starts from rest. Rodas5P and FBDF both use the symbolic Jacobian, so the very
first step from `u0 = 0` returned all-NaN and `dt` collapsed to 5e-324.

What made this hard to see: every cheap diagnostic looks healthy. `u0` and
`du(0)` are exactly zero with no NaN, the *finite-difference* Jacobian is
well-scaled (max entry 329, max |eig| 8.6, no stiffness), the DAE is index-1
(the 2x2 algebraic block for `isα`/`isβ` has det -8.1e-5, full rank), and
`W = M/(γh) - J` is nonsingular at every `dt`. Only the symbolic Jacobian is
poisoned, and only at exactly zero flux — finite differences step off the
singular point and never see it.

Fix in `src/estimators/rotor_flux_observer.jl`: move the existing `flux_eps`
regularization inside the square roots.

    flux_r_mod_obs ~ sqrt(λrα_obs^2 + λrβ_obs^2 + flux_eps^2)
    flux_s_mod_obs ~ sqrt(ψsd_e_obs^2 + ψsq_e_obs^2 + flux_eps^2)

The file already regularized the flux *angle* against `atan(0,0)`; the
*magnitude* had been left unguarded. The offset shifts the magnitude by ~1e-12 Wb
at zero flux and less thereafter — far below any physically meaningful flux.

Verified: analytical Jacobian is NaN-free, `tspan` of 0.05 / 2.0 / 12.0 all
return `Success` (10620 points over the full 12 s), and the controller tracks
its references exactly — `isd` -> 10 A, `isq` -> 15 / -15 / 0 A, rotor flux
settling at 0.4084 Wb = `Lm * isd`. Two regression testsets added: one asserting
the analytical Jacobian is finite at zero flux, one asserting reference
tracking.

### Fixed: `simulate_scalar_im(include_observer = true)` threw

Separate pre-existing bug, found while testing the above. It fails immediately:

    FieldError: type NamedTuple has no field `wsl_obs`

`build_rotor_flux_observer_eqs` reads `vars.wsl_obs` and `vars.ws_obs`
(`rotor_flux_observer.jl:58-59`), but `build_scalar_im_system` never declares
them — its `@variables` block stops at `flux_s_mod_obs`. The FOC system declares
both (`foc_current_im_system.jl:225-226`), so the slip equations were added for
the FOC path and never back-ported to the scalar one. The observer branch of the
scalar system is dead code that errors the moment it is switched on.

Fixed by declaring the two missing variables rather than by deleting the branch:
`scripts/run_scalar_frequency_steps_load_steps.jl` sets `include_observer = true`
and branches on it in five places (variable extraction, an alternative plot
layout, CSV columns, console output), so this was a broken feature rather than
dead code.

Verified: the observer now reconstructs the plant torque to within 1e-4 N m once
the startup transient passes, and rotor flux settles at 0.893 Wb. Regression
testset added.

## Phase 3 — the extension move

Files to move into `ext/IM_AWES_benchMTKExt.jl` (or an `ext/` directory with the
extension including them):

- [x] `src/plants/induction_machine_alpha_beta.jl`
- [x] `src/controls/scalar_vf_control.jl`
- [x] `src/controls/FOC/current_controller.jl` — the continuous one only, *not*
      `current_controller_discrete.jl`
- [x] `src/estimators/rotor_flux_observer.jl` — continuous only, not the
      `_discrete` variant
- [x] `src/profiles/frequency_profiles.jl` — moves wholesale; `*_tstops` is pure
      numeric but is only called from `src/systems/` and is not exported
- [x] `src/profiles/load_profiles.jl` — same
- [x] `src/systems/scalar_im_system.jl`
- [x] `src/systems/foc_current_im_system.jl`

Manifest changes:

- [x] Move `ModelingToolkit` and `OrdinaryDiffEq` from `[deps]` to `[weakdeps]`.
      `OrdinaryDiffEq` matters: leave it in `[deps]` and the solver stack still
      loads on every `using`, which defeats the point.
- [x] Add `[extensions]` with `IM_AWES_benchMTKExt = ["ModelingToolkit", "OrdinaryDiffEq"]`.
- [x] Keep both under `[compat]`.

Stub declarations — an extension can only add methods to functions the parent
already declares, so without these `using ModelingToolkit` yields an
`UndefVarError` rather than the builders:

- [x] In `src/IM_AWES_bench.jl`, declare `function build_scalar_im_system end`,
      `function simulate_scalar_im end`, `function build_foc_current_im_system end`,
      `function simulate_foc_current_im end`.
- [x] Keep the existing `export` lines for those four in the main module.
- [x] Decide whether the six equation builders need stubs too. Answer: no.
      They are unexported and every call site moved into the extension with
      them, so they are plain internal functions of the extension module.
- [x] Delete `using ModelingToolkit` / `using OrdinaryDiffEq` and the include
      lines for moved files from `src/IM_AWES_bench.jl`.
- [x] Add `using ModelingToolkit: t_nounits as t, D_nounits as D` inside the
      extension — the moved builders depend on `D` being in scope.


Layout: `ext/IM_AWES_benchMTKExt.jl` is the extension module and the eight moved
files keep their original subdirectory structure underneath it
(`ext/systems/`, `ext/profiles/`, ...). Moved with `git mv`, so history follows.

Two things the original sketch missed, both found during the move:

- **The extension triggers on ModelingToolkit AND OrdinaryDiffEq**, not MTK
  alone. Julia loads an extension only when *every* package in its trigger list
  is present, and the `simulate_*` functions default to `Rodas5P()` and call
  `solve`, so OrdinaryDiffEq cannot be dropped from the list. The user-facing
  incantation is therefore `using IM_AWES_bench, ModelingToolkit, OrdinaryDiffEq`
  — not the `using ModelingToolkit` alone that was originally assumed. A test
  pins this behaviour explicitly.
- **`build_scalar_im_model` needed moving, not just stubbing.** It was a plain
  binding (`build_scalar_im_model = build_scalar_im_system`) at the bottom of
  `scalar_im_system.jl`, and it is exported. Same rule as the stubs: an
  extension cannot create a binding in its parent, so the alias now lives in
  `src/IM_AWES_bench.jl` as a `const` pointing at the stub. It picks up the
  extension's method automatically.

## Phase 4 — verify

- [x] Phase 2 tests pass: **77 pass, 0 fail, 0 broken**. All three load-isolation
      tripwires flipped from `@test_broken` to passing on their own, via the
      `EXT_MIGRATED` auto-detection.
- [x] `@time using IM_AWES_bench` improved — see the table. 5.67 s -> 0.0029 s.
- [x] Scripts that use only hybrid simulators do not load MTK: verified that
      `using IM_AWES_bench, CSV, DataFrames` under `--project=scripts` leaves
      `ModelingToolkit` absent from `Base.loaded_modules`.
- [x] The scalar/FOC-current path still runs with the extension loaded: builder
      method count is 1 under `--project=examples`, and the full MTK smoke tests
      pass.
- [x] `docs/` updated for the new load requirement
      (`README_IM_AWES_bench_updated.md`, `README_IM_AWES_bench_jl.md`).
- [x] The four scripts that use the MTK path
      (`examples/run_scalar_im.jl`, `scripts/run_scalar_frequency_steps.jl`,
      `scripts/run_scalar_frequency_steps_load_steps.jl`,
      `scripts/run_foc_current_steps.jl`) gained `using ModelingToolkit` and
      `using OrdinaryDiffEq`, and both packages were added to
      `scripts/Project.toml`, `examples/Project.toml` and `test/Project.toml`.
      Not executed — they open plot windows — so this is verified by resolution
      and imports, not by a full run.

## Load time

| Stage | `@time using IM_AWES_bench` |
| --- | --- |
| baseline | 5.73 s, 10.60 M allocations, 738.7 MiB |
| after phase 1 | 5.67 s, 10.54 M allocations, 737.6 MiB |
| after phase 3 | **0.0029 s, 12.29 k allocations, 1.0 MiB** |
| after phase 3, `+ using ModelingToolkit, OrdinaryDiffEq` | 6.27 s |

Phase 3 is where the win landed: a bare `using IM_AWES_bench` went from 5.67 s
to 2.9 ms, and from 10.54 M allocations to 12.29 k — roughly a 2000x cut, since
loading the package now touches nothing but plain Julia. Pulling the extension
in costs 6.27 s, slightly more than the old unconditional load, which is the
expected trade: the symbolic stack is not cheaper, it is just no longer
mandatory.

Phase 1 bought no load-time improvement, and in hindsight could not have: the
six removed packages were declared in `[deps]` but never `using`'d, so they were
never loaded into the session to begin with. Removing them shrinks the install
and resolve footprint — four packages left the manifest entirely — but the
runtime load cost was always `ModelingToolkit` plus `OrdinaryDiffEq`, which are
the only two things `src/IM_AWES_bench.jl` actually imports. The entire load-time
win therefore rests on phase 3.

## Open questions

- Does anything outside this repo depend on the MTK builders being available
  from a bare `using IM_AWES_bench`? If so this is a breaking change and wants a
  version bump beyond `1.0.0-DEV`.
- ~~`examples/` was not inspected.~~ Resolved: `examples/run_scalar_im.jl:11`
  calls `simulate_scalar_im`, so it *does* use the MTK path and will need
  `using ModelingToolkit` added after phase 3. It is the only example.
- ~~Is the split worth a separate package?~~ Resolved by doing it: the
  extension keeps one package and one version, and the load-time result makes
  a second package unnecessary.
- The `[compat]` bounds added for the weakdeps (`ModelingToolkit = "11"`,
  `OrdinaryDiffEq = "7"`) match what is installed. Widen or tighten as needed.
