# Plan: Documenter.jl documentation for `IM_AWES_bench`, published on GitHub Pages

Goal: turn the four hand-written markdown files in `docs/` plus the docstrings in
`src/` and `ext/` into a browsable HTML manual, built by
[Documenter.jl](https://documenter.juliadocs.org/stable/), deployed automatically by
GitHub Actions to

```
https://c-nicomar.github.io/IM_AWES_bench.jl/dev/       # built from main
https://c-nicomar.github.io/IM_AWES_bench.jl/stable/    # built from the latest git tag
```

Status: **not started.** This document is analysis + plan only. Nothing in the
repository has been changed.

Scope: adds `docs/make.jl`, `docs/Project.toml`, `docs/src/`, one GitHub Actions
workflow and one `bin/` helper. Touches `src/`/`ext/` only to add docstrings, and
`README.md` only to add a badge and re-point links. No numerical behaviour changes.

---

## 1. Verified facts about the current tree

Checked against the working tree at commit `b6b94ba` (branch `main`). Re-check if
the code moves under you.

- **There is no `docs/make.jl`, no `docs/src/`, no `docs/Project.toml`.** `docs/`
  currently holds exactly four loose markdown files:
  `README_IM_AWES_bench_jl.md`, `README_IM_AWES_bench_updated.md`,
  `README_MTPA_INTEGRATION.md`, `README_FOC_F1_160KW.md`.
  Documenter requires every page to live under `docs/src/`, so these have to move.
- **There is no `.github/` directory at all** — no CI of any kind exists yet.
- The GitHub remote is `https://github.com/c-nicomar/IM_AWES_bench.jl`, so the
  Pages URL is under `c-nicomar.github.io/IM_AWES_bench.jl`. Note the repository
  name carries the `.jl` suffix while the package directory does not.
- **The package `[deps]` is empty** and must stay empty (see `CLAUDE.md` and
  `oldplans/PlanPackageExtension.md`). Documenter must therefore never appear in
  the root `Project.toml` — only in a separate `docs/Project.toml`.
- The root project declares `[workspace] projects = ["scripts", "test"]`.
  `scripts/` and `test/` each carry `[sources] IM_AWES_bench = {path = ".."}`.
- `Manifest-v1.12.toml` is gitignored; `Manifest-v1.12.toml.default` is committed
  and restored by `bin/install`, which then runs `Pkg.instantiate()` on the
  workspace. `bin/install` also deletes `Manifest.toml` from the root, `scripts/`
  and `test/`.
- `ModelingToolkit` and `OrdinaryDiffEq` are `[weakdeps]` gating the
  `IM_AWES_benchMTKExt` extension. Four exported names
  (`build_scalar_im_system`, `simulate_scalar_im`, `build_foc_current_im_system`,
  `simulate_foc_current_im`) are **empty stubs** in `src/IM_AWES_bench.jl` and get
  their methods only from `ext/`.
- **Docstring coverage is poor.** Counting `"""` blocks per file:
  - `ext/` — every file has at least one; the profile/system builders have several.
  - `src/` — only `src/simulators/hybrid_foc_speed_f1_160kw_simulator.jl` has any.
    All four controllers in `src/controls/FOC/`, both estimators in
    `src/estimators/`, and four of the five simulators have **zero** docstrings,
    despite exporting ~40 names between them.
  - `src/IM_AWES_bench.jl:2-4` contains a stray `"""bla"""` string literal *inside*
    the module body. It is not the module docstring (a module docstring must
    precede the `module` keyword) and it does not attach to anything — verified.
    It should be deleted or replaced by a real module docstring above
    `module IM_AWES_bench`.
- Julia 1.12.6 is the active version; `bin/install` hard-requires 1.12.

**Measured after the first build (2026-07-29), superseding the `"""` counts
above.** Those counts were of `"""` occurrences, which over-counts: a docstring
opens and closes with one each, and some blocks are ordinary strings inside
function bodies. The real numbers, read from the loaded modules:

- `length(names(IM_AWES_bench))` = **37** exported names.
- `length(Docs.meta(IM_AWES_bench))` = **3** attached docstrings —
  `simulate_foc_speed_f1_im_160kw`, `im_160kw_speed_reference`,
  `im_160kw_load_torque_profile`. Everything else exported is undocumented.
- `length(Docs.meta(MTKExt))` = **13**. The extension is the well-documented half
  of the package: the 4 public builders (docstrings written in `ext/systems/`,
  keyed to the `IM_AWES_bench.*` bindings they add methods to) plus 8 internal
  equation builders plus the module docstring.
- Combined: **36 exports excluding the module name, 7 documented, 29
  undocumented.** All 29 are in `src/` — the discrete controllers, the
  estimators, and four of the five hybrid simulators.

An earlier revision of this section claimed the `ext/` prose blocks "did not
register as docstrings". That was wrong. They are ordinary attached docstrings;
they simply never appeared in the build warning because `checkdocs = :exports`
filters each module's docstrings down to names *that module* exports, and
neither the internal builders nor a parent-module binding documented from within
the extension qualifies.

**Consequence — corrected.** The original claim here ("`@autodocs` will trigger
Documenter's missing-docs check and fail the build") was wrong about what that
check does. Documenter 1.x `checkdocs` compares *docstrings that exist* against
docstrings *spliced into the manual*; it warns about "docstrings not included in
the manual". It has no coverage gate for an exported name that has no docstring
at all — those are invisible to it. The verified first build emitted exactly one
such warning, naming the 4 docstrings above, and said nothing whatsoever about
the other 34 exports.

Two things follow:

1. `warnonly = [:missing_docs]` turned out to be unnecessary. Once the API pages
   existed, every docstring in the package had a home and the build was already
   strict-clean, so the setting was dropped rather than carried. **Done.**
2. `checkdocs = :exports` does **not** turn missing docstrings into build
   failures. It only guarantees that every docstring that exists is published.
   Docstring *coverage* has to be enforced by review, or by an explicit test
   comparing `names()` against `Docs.meta()`.

---

## 2. Design decisions, with rationale

### 2.1 `docs/` is *not* a workspace member

Tempting, for consistency with `scripts/` and `test/`. Don't do it.

Adding `"docs"` to `[workspace] projects` folds Documenter and its whole
dependency tree into the shared `Manifest-v1.12.toml`. That manifest is restored
verbatim from the committed `Manifest-v1.12.toml.default` by `bin/install`, which
then calls `Pkg.instantiate()`. A `.default` that predates the docs environment no
longer describes the workspace, and instantiate fails or silently re-resolves —
breaking the one command every contributor runs first, for the benefit of a build
almost nobody runs locally.

Instead `docs/` gets a standalone `Project.toml` with `[sources]`, exactly the
`scripts`/`test` pattern minus workspace membership. Its `docs/Manifest.toml`
resolves independently and is already covered by the `Manifest.toml` line in
`.gitignore`.

*If* you later want the docs env in the workspace, the price is regenerating and
committing `Manifest-v1.12.toml.default` in the same commit. Decide once; don't
half-do it.

### 2.2 The docs build loads ModelingToolkit and OrdinaryDiffEq

The four MTK-backed functions are zero-method stubs unless both triggers are
loaded. Documenter documents what is in the session, so:

- If `make.jl` only does `using IM_AWES_bench`, the docstrings written in `ext/`
  are invisible and the four stubs render as bare, undocumented functions.
- If `make.jl` does `using IM_AWES_bench, ModelingToolkit, OrdinaryDiffEq`, the
  extension loads and everything is documented — at the cost of an MTK
  precompilation in every docs build (several minutes cold, cached in CI by
  `julia-actions/cache`).

**Take the cost.** Half the package's public surface is otherwise undocumented.
Document the extension's own docstrings via `@autodocs` with
`Modules = [Base.get_extension(IM_AWES_bench, :IM_AWES_benchMTKExt)]`.

### 2.3 Existing markdown files move rather than get rewritten

The four `docs/README_*.md` files are good prose and already carry the
authoritative sign conventions. Move them into `docs/src/` under shorter names,
fix relative links, and list them in `pages`. Do not rewrite them in this step —
that is a separate content task.

### 2.4 Deploy with `GITHUB_TOKEN`, not an SSH deploy key

`DocumenterTools.genkeys` + `DOCUMENTER_KEY` is the older path and still works,
but it means generating a keypair, adding a deploy key with write access, and
adding a repository secret. A workflow with `permissions: contents: write` and the
built-in `GITHUB_TOKEN` needs zero secrets. The only thing it cannot do is deploy
previews for pull requests from forks; for this repository that is acceptable.

---

## 3. Target file layout

```
.github/
  workflows/
    Documenter.yml          # new
bin/
  build_docs                # new, optional helper
docs/
  .gitignore                # new: build/, Manifest.toml
  Project.toml              # new
  make.jl                   # new
  src/
    index.md                # new: landing page
    architecture.md         # <- docs/README_IM_AWES_bench_jl.md
    overview.md             # <- docs/README_IM_AWES_bench_updated.md
    mtpa.md                 # <- docs/README_MTPA_INTEGRATION.md
    foc_f1_160kw.md         # <- docs/README_FOC_F1_160KW.md
    scripts.md              # new: what each scripts/run_*.jl does
    api/
      package.md            # new: covers src/IM_AWES_bench.jl itself
      controllers.md        # new
      estimators.md         # new
      simulators.md         # new
      mtk.md                # new: the extension surface
```

The old `docs/README_*.md` files are `git mv`-ed, not copied — two sources of
truth for the sign conventions is exactly the failure mode `CLAUDE.md` warns
about (it still describes a `src/profiles/` that no longer exists).

---

## 4. Step-by-step

### Step 1 — create the docs environment

`docs/Project.toml`:

```toml
[deps]
Documenter = "e30172f5-a6a5-5a46-863b-614d45cd2de4"
IM_AWES_bench = "d85c8359-f9b8-4611-8006-e67b7a824205"
ModelingToolkit = "961ee093-0014-501f-94e3-6117800e7a78"
OrdinaryDiffEq = "1dea7af3-3e70-54e6-95c3-0bf5283fa5ed"

[sources]
IM_AWES_bench = {path = ".."}

[compat]
Documenter = "1"
ModelingToolkit = "11"
OrdinaryDiffEq = "7"
julia = "1.12"
```

`docs/.gitignore`:

```
build/
Manifest.toml
```

(The root `.gitignore` already ignores `Manifest.toml` everywhere; the local file
is belt-and-braces and also covers `build/`, which the root file does **not**
ignore today. Alternatively add `docs/build/` to the root `.gitignore` — pick one.)

Resolve it once:

```bash
julia --project=docs -e 'using Pkg; Pkg.instantiate()'
```

### Step 2 — move the existing pages

```bash
git mv docs/README_IM_AWES_bench_jl.md      docs/src/architecture.md
git mv docs/README_IM_AWES_bench_updated.md docs/src/overview.md
git mv docs/README_MTPA_INTEGRATION.md      docs/src/mtpa.md
git mv docs/README_FOC_F1_160KW.md          docs/src/foc_f1_160kw.md
```

Then fix links inside them:

- Cross-references between the four files become `[text](mtpa.md)` — Documenter
  rewrites `.md` links to the generated `.html`.
- Links that pointed at source files (`src/simulators/…`, `scripts/…`) are now two
  directories deeper and, more importantly, point at files that are not part of
  the doc build. Rewrite them as absolute GitHub URLs, e.g.
  `https://github.com/c-nicomar/IM_AWES_bench.jl/blob/main/src/simulators/hybrid_foc_speed_f1_simulator.jl`.
  Anything left as a broken relative link is reported by `checkdocs`/`linkcheck`.

Grep for what needs fixing:

```bash
grep -n '](\(\.\./\|src/\|ext/\|scripts/\|docs/\|oldplans/\|profiles/\)' docs/src/*.md
```

### Step 3 — write `docs/src/index.md`

Landing page. Content: the one-paragraph purpose from `README.md`, the two
modelling paths (hybrid FOC vs. MTK), the install/`bin/install` quickstart, the
`using IM_AWES_bench` vs. `using IM_AWES_bench, ModelingToolkit, OrdinaryDiffEq`
distinction, and — prominently — the sign convention:

```
J * dω/dt = Te + TL - B * ω
```

Add a table of contents:

````markdown
```@contents
Pages = ["overview.md", "architecture.md", "mtpa.md", "foc_f1_160kw.md", "scripts.md"]
Depth = 2
```
````

### Step 4 — write `docs/make.jl`

```julia
using Documenter
using IM_AWES_bench
# Both triggers, so IM_AWES_benchMTKExt loads and its docstrings become visible.
using ModelingToolkit, OrdinaryDiffEq

const MTKExt = Base.get_extension(IM_AWES_bench, :IM_AWES_benchMTKExt)
MTKExt === nothing && error("IM_AWES_benchMTKExt did not load — check the weakdep triggers")

DocMeta.setdocmeta!(IM_AWES_bench, :DocTestSetup, :(using IM_AWES_bench); recursive = true)

makedocs(;
    modules  = [IM_AWES_bench, MTKExt],
    authors  = "Carolina Nicolás <canicola@ing.uc3m.es> and contributors",
    sitename = "IM_AWES_bench.jl",
    format = Documenter.HTML(;
        canonical = "https://c-nicomar.github.io/IM_AWES_bench.jl",
        edit_link = "main",
        prettyurls = get(ENV, "CI", "false") == "true",
        assets = String[],
    ),
    pages = [
        "Home" => "index.md",
        "Package overview" => "overview.md",
        "Architecture" => "architecture.md",
        "MTPA integration" => "mtpa.md",
        "160 kW FOC F1 case" => "foc_f1_160kw.md",
        "Example scripts" => "scripts.md",
        "API" => [
            "Controllers"    => "api/controllers.md",
            "Estimators"     => "api/estimators.md",
            "Simulators"     => "api/simulators.md",
            "MTK extension"  => "api/mtk.md",
        ],
    ],
    checkdocs = :exports,
)

deploydocs(;
    repo = "github.com/c-nicomar/IM_AWES_bench.jl",
    devbranch = "main",
    push_preview = true,
)
```

Notes:

- `prettyurls` off locally gives clickable `file://` output; on in CI gives clean
  URLs. This is the standard idiom.
- `repo` in `deploydocs` must be the **GitHub repository** name including `.jl`,
  with no scheme and no `.git` — mismatches here are the single most common
  reason a build succeeds but nothing is published.
- No `warnonly`. The API pages of Step 5 cover every file in `src/` and `ext/`,
  so nothing is orphaned and the build is strict-clean as written. There is also
  an `api/package.md` page whose only job is to cover `src/IM_AWES_bench.jl`,
  so the module docstring and the `build_scalar_im_model` alias have a home when
  they are written.

### Step 5 — API pages and the docstring gap

Each API page selects docstrings by source file, which keeps the page order
meaningful and avoids one giant alphabetical dump. Example
`docs/src/api/controllers.md`:

````markdown
# Controllers

```@meta
CurrentModule = IM_AWES_bench
```

## Inner current controller

```@autodocs
Modules = [IM_AWES_bench]
Pages   = ["controls/FOC/current_controller_discrete.jl"]
```

## Outer loops

```@autodocs
Modules = [IM_AWES_bench]
Pages   = ["controls/FOC/outer_torque_flux_f1_discrete.jl",
           "controls/FOC/outer_speed_flux_f1_discrete.jl",
           "controls/FOC/outer_speed_flux_mtpa_discrete.jl"]
```
````

`Pages` entries are matched as **suffixes of the source path**, so
`"controls/FOC/current_controller_discrete.jl"` is the right granularity — a bare
basename would also match the `ext/controls/FOC/current_controller.jl` docstrings.

`docs/src/api/mtk.md` uses the extension module and must state the loading
contract in prose, because a reader who only did `using IM_AWES_bench` will hit a
zero-method `MethodError`:

````markdown
# ModelingToolkit extension

These builders exist only when **both** `ModelingToolkit` and `OrdinaryDiffEq`
are loaded. A `MethodError` listing no methods means a trigger is missing.

```@autodocs
Modules = [Base.get_extension(IM_AWES_bench, :IM_AWES_benchMTKExt)]
```
````

**The docstring pass.** This is the largest chunk of real work in the plan:
**29 of the 36 exported names have no docstring at all** (measured — see §1), and
all 29 are in `src/`. Nothing in the toolchain will nag you about that, so it has
to be driven by this list rather than by build failures. The API pages are
already in place, so each docstring appears on the site the moment it is written.
Order of attack:

1. `src/IM_AWES_bench.jl` — delete the stray `"""bla"""` at line 2 and put a real
   module docstring **above** `module IM_AWES_bench`. Also document the
   `build_scalar_im_model` alias. Both land on `api/package.md`.
2. The four `simulate_*_hybrid` entry points plus
   `simulate_foc_speed_f1_im_160kw` — highest reader value. (The 160 kW one is
   already documented; the other four are not.)
3. The six `*_step!` functions, each with its `*State` / `*Params` / `*Output`
   trio — 24 names, the bulk of the remainder. Document the `*_step!` function
   fully (arguments, units, sign conventions) and give each struct a one-line
   docstring plus per-field notes.

Every docstring that mentions torque must match `J*dω/dt = Te + TL - B*ω`.
Cross-check against sections 3, 9 and 11 of `docs/src/architecture.md`.

Be clear about what the strict `checkdocs = :exports` already in `make.jl` buys:
it makes the build fail when a docstring exists but is not spliced into the
manual. It does *not* catch a newly added export that has no docstring. If you
want coverage genuinely enforced, add a testset to `test/runtests.jl` along the
lines of

```julia
undocumented = setdiff(names(IM_AWES_bench), [:IM_AWES_bench], keys(Docs.meta(IM_AWES_bench)))
@test isempty(undocumented)
```

adjusting for the fact that `Docs.meta` is keyed by `Docs.Binding`, not `Symbol`.

### Step 6 — build and preview locally

```bash
julia --project=docs docs/make.jl
```

Output lands in `docs/build/`; open `docs/build/index.html`. `deploydocs` is a
no-op outside CI, so this is safe.

Live-reloading preview:

```bash
julia --project=docs -e 'using Pkg; Pkg.add("LiveServer")'
julia --project=docs -e 'using LiveServer; servedocs()'
```

Optional `bin/build_docs`, matching the style of the other `bin/` scripts:

```bash
#!/bin/bash
set -eu
if [[ $(basename $(pwd)) == "bin" ]]; then cd ..; fi
export KMP_DUPLICATE_LIB_OK="TRUE"
julia --project=docs -e 'using Pkg; Pkg.instantiate()'
julia --project=docs docs/make.jl
echo "Open docs/build/index.html"
```

`chmod +x bin/build_docs`. Note it deliberately does **not** use `bin/sysimage.so`
— the sysimage bakes in MTK and has already caused extension-visibility problems
in the test suite.

Expect the first local build to take a while (MTK precompilation), a few tens of
seconds after that.

### Step 7 — the GitHub Actions workflow

`.github/workflows/Documenter.yml`:

```yaml
name: Documentation

on:
  push:
    branches: [main]
    tags: ['*']
  pull_request:
  workflow_dispatch:

concurrency:
  group: ${{ github.workflow }}-${{ github.ref }}
  cancel-in-progress: true

jobs:
  docs:
    name: Build and deploy documentation
    runs-on: ubuntu-latest
    permissions:
      contents: write        # push to gh-pages
      pull-requests: write   # PR preview comments
      statuses: write
    steps:
      - uses: actions/checkout@v4
      - uses: julia-actions/setup-julia@v2
        with:
          version: '1.12'
      - uses: julia-actions/cache@v2
      - name: Instantiate docs environment
        run: julia --project=docs -e 'using Pkg; Pkg.instantiate()'
      - name: Build and deploy
        env:
          GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}
          JULIA_DEBUG: Documenter
          KMP_DUPLICATE_LIB_OK: 'TRUE'
        run: julia --project=docs docs/make.jl
```

Points that matter:

- `version: '1.12'` mirrors the `bin/install` requirement; `'1'` would drift to
  whatever is current.
- `KMP_DUPLICATE_LIB_OK` is exported by `bin/install` and `bin/run_julia` for a
  reason — carry it into CI.
- No `DOCUMENTER_KEY`. The built-in `GITHUB_TOKEN` plus `contents: write` is
  enough, given the `permissions:` block above.
- Do not add `julia-actions/julia-buildpkg` — it targets the root project, whose
  `[deps]` are empty; `Pkg.instantiate()` on `docs/` already develops the package
  through `[sources]`.

### Step 8 — one-time GitHub configuration

1. **Allow Actions to write.** Repository → Settings → Actions → General →
   *Workflow permissions* → "Read and write permissions". Without this the
   `permissions:` block cannot grant `contents: write` and the deploy silently
   does nothing.
2. **Merge to `main`** and watch the run under the Actions tab. On success it
   creates an orphan `gh-pages` branch containing `dev/` and a `versions.js`.
3. **Enable Pages.** Settings → Pages → *Source*: "Deploy from a branch",
   branch `gh-pages`, folder `/ (root)`. Save. First publish takes a minute or two.
4. **Verify** `https://c-nicomar.github.io/IM_AWES_bench.jl/dev/`.
5. **`stable/` requires a tag.** `/stable/` only appears after a version tag is
   pushed (`v1.0.0`, matching `version = "1.0.0"` in `Project.toml`) and the
   tag-triggered run completes. Until then, link to `/dev/`.

If step 2 produces "Deploy: skipped" in the log, read the `JULIA_DEBUG=Documenter`
output — it prints exactly which of repo/branch/token/event checks failed.

### Step 9 — wire the docs into the repository

- Badge at the top of `README.md`:

  ```markdown
  [![Dev](https://img.shields.io/badge/docs-dev-blue.svg)](https://c-nicomar.github.io/IM_AWES_bench.jl/dev/)
  [![Stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://c-nicomar.github.io/IM_AWES_bench.jl/stable/)
  ```

- Replace the four `docs/README_*.md` links in the "Detailed documentation"
  section of `README.md` with links to the hosted pages.
- Update `CLAUDE.md`: the *Documentation* list points at the four old paths, and
  the *Commands* section should gain `bin/build_docs`. While in there, fix the
  stale `src/profiles/` claim flagged in `PlanRefactoring.md`.
- `oldplans/` and `PlanRefactoring.md` stay out of the doc build — they are
  historical.

---

## 5. Optional follow-ups, in priority order

1. **A test CI workflow.** There is none. `test/runtests.jl` spawns subprocesses
   to verify extension loading and is the actual contract for the zero-dep
   design; it currently only ever runs on a developer's laptop. Higher value than
   anything below.
2. **Doctests.** `jldoctest` blocks in docstrings would be executed by
   `makedocs`. Good fit for the pure `*_step!` functions with small inputs; a bad
   fit for the simulators, whose outputs are long float vectors. Note that any
   doctest touching MTK will be slow.
3. **`linkcheck = true`** in `makedocs` — catches the GitHub URLs introduced in
   Step 2 going stale. Enable only after the workflow is green, since it makes
   the build network-dependent and flaky.
4. **A `scripts.md` page generated from the script headers** rather than
   hand-maintained — deferred until `PlanRefactoring.md` lands, since that plan
   restructures `scripts/` anyway.
5. **Plots in the docs.** The scripts plot with `MakieControlPlots`. Embedding
   generated figures means adding Makie to the docs environment and a headless
   backend in CI — a real cost. Consider committing a handful of static PNGs
   under `docs/src/assets/` instead.

---

## 6. Pitfalls specific to this repository

| Pitfall | Symptom | Avoidance |
| --- | --- | --- |
| Adding Documenter to the root `Project.toml` | Package gains a hard dep; the extension-contract testsets in `test/runtests.jl` fail | Documenter only in `docs/Project.toml` |
| Adding `"docs"` to `[workspace] projects` without regenerating `Manifest-v1.12.toml.default` | `bin/install` fails at `Pkg.instantiate()` for everyone | Keep `docs/` outside the workspace (§2.1) |
| A stray `docs/Manifest.toml` committed | Shadows nothing (docs is not a workspace member) but pins stale versions in CI | Covered by `.gitignore`; `bin/install` does *not* clean it, so do not create it by hand |
| `make.jl` forgetting `using OrdinaryDiffEq` | Extension never loads; four exported names render as undocumented stubs | The `Base.get_extension(...) === nothing && error(...)` guard in Step 4 fails loudly |
| Building docs under `bin/sysimage.so` | Extension-visibility weirdness, same class of failure as the test suite hits | `bin/build_docs` calls plain `julia`, never `-J` |
| `deploydocs(repo = "github.com/c-nicomar/IM_AWES_bench")` | Build green, nothing ever published | The repository name includes `.jl` |
| `devbranch` left at the default `"master"` | Deploy skipped on every push | `devbranch = "main"` |
| An `@autodocs` page added without covering every source file | Orphaned docstrings fail the strict build | Keep the API pages' `Pages` filters exhaustive over `src/` and `ext/` |
| Trusting `checkdocs = :exports` as a coverage gate | 34 undocumented exports pass the build in silence | Enforce coverage by review or a `names()` vs `Docs.meta()` test (Step 5) |

---

## 7. Checklist

- [ ] `docs/Project.toml` + `docs/.gitignore` created, environment instantiates
- [x] Four `README_*.md` files `git mv`-ed into `docs/src/`, internal links fixed
- [x] `docs/src/index.md` written
- [x] `docs/make.jl` written, extension guard passes
- [x] Five `docs/src/api/*.md` pages written (`package` was added to the four
      planned, so `src/IM_AWES_bench.jl` is covered)
- [x] `julia --project=docs docs/make.jl` builds clean locally — 10 pages, no
      warnings beyond the expected local "skipping deployment"
- [x] `bin/build_docs` added and executable; verified from a deleted
      `docs/build/`
- [ ] `.github/workflows/Documenter.yml` added
- [ ] Workflow permissions set to read/write in repository settings
- [ ] First `main` build green, `gh-pages` branch created
- [ ] Pages source set to `gh-pages` / root, `/dev/` reachable
- [ ] `README.md` badges + links updated; `CLAUDE.md` updated
- [ ] Docstring pass complete (29 exports); build still green
- [ ] `v1.0.0` tagged so `/stable/` exists

---

## 8. Effort estimate

| Work | Estimate |
| --- | --- |
| Steps 1–4, 6–8 (environment, page moves, `make.jl`, workflow, GitHub setup) | half a day, most of it waiting on builds |
| Step 5 API pages (skeletons) | done — took about an hour |
| Step 5 docstring pass (29 exported names, sign conventions checked) | 1–2 days, and the only part worth doing carefully |
| Step 9 README/CLAUDE.md rewiring | under an hour |

The site can be live and deploying at the end of day one; the docstrings are what
make it worth visiting.
