"""
    build_scalar_vf_control_eqs(vars, pars)

Builds the scalar V/f control equations.

This block is the controller / ideal voltage source.

Input:
- `f_cmd`

Outputs:
- `va`, `vb`, `vc`
- `vsα`, `vsβ`

It does not contain induction machine physics.
"""
function build_scalar_vf_control_eqs(vars, pars)

    f_cmd = vars.f_cmd
    θs = vars.θs
    Vphase_peak = vars.Vphase_peak

    va = vars.va
    vb = vars.vb
    vc = vars.vc

    vsα = vars.vsα
    vsβ = vars.vsβ

    Vll_nom = pars.Vll_nom
    f_nom = pars.f_nom

    return [
        # Electrical angle from commanded stator frequency
        D(θs) ~ 2π * f_cmd,

        # V/f law
        Vphase_peak ~ sqrt(2) * Vll_nom / sqrt(3) * f_cmd / f_nom,

        # Balanced three-phase voltage references
        va ~ Vphase_peak * sin(θs),
        vb ~ Vphase_peak * sin(θs - 2π / 3),
        vc ~ Vphase_peak * sin(θs + 2π / 3),

        # abc to stationary alpha-beta transformation
        vsα ~ 2 / 3 * (va - 0.5 * vb - 0.5 * vc),
        vsβ ~ 2 / 3 * (sqrt(3) / 2 * (vb - vc)),
    ]
end