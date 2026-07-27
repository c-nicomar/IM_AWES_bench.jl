"""
    build_rotor_flux_observer_eqs(vars, pars)

Rotor-flux and torque observer for the induction machine.

This is based on the MATLAB function `obs_flujo_L1_alfabeta`, adapted to
continuous-time ModelingToolkit equations.

Important robustness detail:
At startup, rotor flux is zero, so the flux angle is undefined. To avoid a
singular atan2(0,0), the angle is regularized with a small epsilon.
"""
function build_rotor_flux_observer_eqs(vars, pars)

    # ------------------------------------------------------------
    # Plant / measured inputs
    # ------------------------------------------------------------

    isα = vars.isα
    isβ = vars.isβ
    θm = vars.θm
    ωm = vars.ωm

    # ------------------------------------------------------------
    # Observer states
    # ------------------------------------------------------------

    λrd_r_obs = vars.λrd_r_obs
    λrq_r_obs = vars.λrq_r_obs

    # ------------------------------------------------------------
    # Observer algebraic variables
    # ------------------------------------------------------------

    θr_obs = vars.θr_obs

    isd_r_obs = vars.isd_r_obs
    isq_r_obs = vars.isq_r_obs

    λrα_obs = vars.λrα_obs
    λrβ_obs = vars.λrβ_obs

    flux_r_mod_obs = vars.flux_r_mod_obs
    θe_obs = vars.θe_obs

    isd_e_obs = vars.isd_e_obs
    isq_e_obs = vars.isq_e_obs

    λrd_e_obs = vars.λrd_e_obs
    λrq_e_obs = vars.λrq_e_obs

    Te_obs = vars.Te_obs

    ψsd_e_obs = vars.ψsd_e_obs
    ψsq_e_obs = vars.ψsq_e_obs
    flux_s_mod_obs = vars.flux_s_mod_obs

    wsl_obs = vars.wsl_obs
    ws_obs = vars.ws_obs

    # ------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------

    Lm = pars.Lm
    Lss = pars.Lss
    Lrr = pars.Lrr
    tau_r = pars.tau_r
    p = pars.p

    # Small numerical regularization for zero-flux startup
    flux_eps = 1e-6

    return [
        # --------------------------------------------------------
        # Rotor electrical angle
        # --------------------------------------------------------
        θr_obs ~ p * θm,

        # --------------------------------------------------------
        # Park transform: stationary αβ -> rotor frame dq_r
        # --------------------------------------------------------
        isd_r_obs ~  cos(θr_obs) * isα + sin(θr_obs) * isβ,
        isq_r_obs ~ -sin(θr_obs) * isα + cos(θr_obs) * isβ,

        # --------------------------------------------------------
        # Rotor-flux observer in rotor reference frame
        #
        # Continuous equivalent of:
        # λrd_r[k+1] = α λrd_r[k] + β Lm i_sd_r[k]
        # --------------------------------------------------------
        D(λrd_r_obs) ~ -(1 / tau_r) * λrd_r_obs + (Lm / tau_r) * isd_r_obs,
        D(λrq_r_obs) ~ -(1 / tau_r) * λrq_r_obs + (Lm / tau_r) * isq_r_obs,

        # --------------------------------------------------------
        # Rotor flux back to stationary αβ
        # --------------------------------------------------------
        λrα_obs ~ cos(θr_obs) * λrd_r_obs - sin(θr_obs) * λrq_r_obs,
        λrβ_obs ~ sin(θr_obs) * λrd_r_obs + cos(θr_obs) * λrq_r_obs,

        # --------------------------------------------------------
        # Rotor flux magnitude and robust angle
        #
        # Instead of atan(0,0), use atan(λβ, λα + eps).
        # This gives θe = 0 at zero flux and avoids a singular initial DAE.
        # --------------------------------------------------------
        # The epsilon inside the sqrt is not cosmetic. sqrt(x) has an infinite
        # derivative at x = 0, so the symbolic Jacobian of the unregularized
        # form is λrα/sqrt(λrα^2 + λrβ^2) = 0/0 = NaN at zero flux. Solvers
        # that use MTK's analytical Jacobian (Rodas5P, FBDF) then produce NaN
        # on their very first step from a zero-flux initial condition. The
        # offset shifts the magnitude by ~1e-12 Wb at zero flux and less
        # thereafter, which is far below any physically meaningful flux.
        flux_r_mod_obs ~ sqrt(λrα_obs^2 + λrβ_obs^2 + flux_eps^2),
        θe_obs ~ atan(λrβ_obs, λrα_obs + flux_eps),

        # --------------------------------------------------------
        # Park transform: stationary αβ -> estimated flux frame dq_e
        # --------------------------------------------------------
        isd_e_obs ~  cos(θe_obs) * isα + sin(θe_obs) * isβ,
        isq_e_obs ~ -sin(θe_obs) * isα + cos(θe_obs) * isβ,

        # --------------------------------------------------------
        # Rotor flux in estimated flux frame
        #
        # Since θe_obs is defined from the rotor-flux vector, the d-axis
        # component is the flux magnitude and the q-axis component is zero.
        # This is numerically more robust than projecting using atan at
        # near-zero flux.
        # --------------------------------------------------------
        λrd_e_obs ~ flux_r_mod_obs,
        λrq_e_obs ~ 0.0,

        # --------------------------------------------------------
        # Estimated electromagnetic torque
        # --------------------------------------------------------
        Te_obs ~ 1.5 * p * (Lm / Lrr) * λrd_e_obs * isq_e_obs,

        # --------------------------------------------------------
        # Estimated stator flux in flux frame
        # --------------------------------------------------------
        ψsd_e_obs ~ Lss * isd_e_obs + (Lm / Lrr) * λrd_e_obs,
        ψsq_e_obs ~ Lss * isq_e_obs + (Lm / Lrr) * λrq_e_obs,
        # Same zero-derivative regularization as flux_r_mod_obs above.
        flux_s_mod_obs ~ sqrt(ψsd_e_obs^2 + ψsq_e_obs^2 + flux_eps^2),

        # --------------------------------------------------------
        # Estimated slip and synchronous electrical speed
        #
        # Regularized to avoid division by zero at startup.
        # --------------------------------------------------------
        wsl_obs ~ (1 / tau_r) * (Lm * isq_e_obs) / (λrd_e_obs + flux_eps),
        ws_obs ~ p * ωm + wsl_obs,
    ]
end