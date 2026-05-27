"""
    build_rotor_flux_observer_eqs(vars, pars)

Rotor-flux and torque observer for the induction machine.

This is based on the MATLAB function `obs_flujo_L1_alfabeta`.

Inputs used from the plant:
- `isα`, `isβ`
- `θm`

Estimated outputs:
- `λrd_e_obs`, `λrq_e_obs`
- `θe_obs`
- `Te_obs`
- `flux_r_mod_obs`
- `isd_e_obs`, `isq_e_obs`
- `ψsd_e_obs`, `ψsq_e_obs`
- `flux_s_mod_obs`

The observer is parallel to the plant. It does not affect the plant dynamics.
"""
function build_rotor_flux_observer_eqs(vars, pars)

    # ------------------------------------------------------------
    # Plant/measured inputs
    # ------------------------------------------------------------

    isα = vars.isα
    isβ = vars.isβ
    θm = vars.θm

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

    # ------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------

    Lm = pars.Lm
    Lss = pars.Lss
    Lrr = pars.Lrr
    tau_r = pars.tau_r
    p = pars.p

    return [
        # --------------------------------------------------------
        # Rotor electrical angle
        # --------------------------------------------------------
        θr_obs ~ p * θm,

        # --------------------------------------------------------
        # Park transform: stationary αβ -> rotor frame dq_r
        # Equivalent to MATLAB:
        # i_sd_r =  cos(theta_r)*i_alpha + sin(theta_r)*i_beta
        # i_sq_r = -sin(theta_r)*i_alpha + cos(theta_r)*i_beta
        # --------------------------------------------------------
        isd_r_obs ~  cos(θr_obs) * isα + sin(θr_obs) * isβ,
        isq_r_obs ~ -sin(θr_obs) * isα + cos(θr_obs) * isβ,

        # --------------------------------------------------------
        # Continuous-time equivalent of the L1 rotor-flux observer:
        #
        # MATLAB discrete form:
        # λrd_r[k+1] = α λrd_r[k] + β Lm i_sd_r[k]
        #
        # Continuous form:
        # dλrd_r/dt = -(1/tau_r) λrd_r + (Lm/tau_r) i_sd_r
        # --------------------------------------------------------
        D(λrd_r_obs) ~ -(1 / tau_r) * λrd_r_obs + (Lm / tau_r) * isd_r_obs,
        D(λrq_r_obs) ~ -(1 / tau_r) * λrq_r_obs + (Lm / tau_r) * isq_r_obs,

        # --------------------------------------------------------
        # Rotor flux from rotor frame back to stationary αβ
        # --------------------------------------------------------
        λrα_obs ~ cos(θr_obs) * λrd_r_obs - sin(θr_obs) * λrq_r_obs,
        λrβ_obs ~ sin(θr_obs) * λrd_r_obs + cos(θr_obs) * λrq_r_obs,

        # --------------------------------------------------------
        # Rotor flux magnitude and angle
        #
        # MATLAB includes a guard:
        # if flux_r_mod > 1e-6
        #     theta_e = atan2(...)
        # else
        #     theta_e = 0
        # end
        #
        # Here we use atan directly. At exact zero flux, angle is not
        # physically meaningful anyway, but the simulation should move away
        # from zero once voltage is applied.
        # --------------------------------------------------------
        flux_r_mod_obs ~ sqrt(λrα_obs^2 + λrβ_obs^2),
        θe_obs ~ atan(λrβ_obs, λrα_obs),

        # --------------------------------------------------------
        # Park transform: stationary αβ -> estimated rotor-flux frame dq_e
        # --------------------------------------------------------
        isd_e_obs ~  cos(θe_obs) * isα + sin(θe_obs) * isβ,
        isq_e_obs ~ -sin(θe_obs) * isα + cos(θe_obs) * isβ,

        # --------------------------------------------------------
        # Rotor flux projected into estimated flux frame
        # --------------------------------------------------------
        λrd_e_obs ~  cos(θe_obs) * λrα_obs + sin(θe_obs) * λrβ_obs,
        λrq_e_obs ~ -sin(θe_obs) * λrα_obs + cos(θe_obs) * λrβ_obs,

        # --------------------------------------------------------
        # Estimated electromagnetic torque
        # MATLAB:
        # Tem_est = 1.5 * p * (Lm/Lrr) * lambda_rd_e * i_sq_e
        # --------------------------------------------------------
        Te_obs ~ 1.5 * p * (Lm / Lrr) * λrd_e_obs * isq_e_obs,

        # --------------------------------------------------------
        # Estimated stator flux in flux frame
        # MATLAB:
        # psi_sd_e = Lss*i_sd_e + (Lm/Lrr)*lambda_rd_e
        # psi_sq_e = Lss*i_sq_e + (Lm/Lrr)*lambda_rq_e
        # --------------------------------------------------------
        ψsd_e_obs ~ Lss * isd_e_obs + (Lm / Lrr) * λrd_e_obs,
        ψsq_e_obs ~ Lss * isq_e_obs + (Lm / Lrr) * λrq_e_obs,
        flux_s_mod_obs ~ sqrt(ψsd_e_obs^2 + ψsq_e_obs^2),
    ]
end