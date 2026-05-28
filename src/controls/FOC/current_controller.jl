"""
    build_foc_current_controller_eqs(vars, pars)

Continuous-time simplified FOC current controller for an induction machine.

This block is based on the MATLAB current-controller logic, adapted to the
continuous-time ModelingToolkit model.

Inputs:
- isd_ref, isq_ref
- isd_e_obs, isq_e_obs
- ws_obs
- λrd_e_obs

Outputs:
- vsd_ref, vsq_ref
- vsα, vsβ
- isd_ref_lim, isq_ref_lim
- voltage_saturation
- is_mod_obs

The controller does not contain plant equations.
"""
function build_foc_current_controller_eqs(vars, pars)

    # ------------------------------------------------------------
    # References
    # ------------------------------------------------------------

    isd_ref = vars.isd_ref
    isq_ref = vars.isq_ref

    # ------------------------------------------------------------
    # Measurements from observer
    # ------------------------------------------------------------

    isd_med = vars.isd_e_obs
    isq_med = vars.isq_e_obs
    omega_e = vars.ws_obs
    lambda_rd = vars.λrd_e_obs
    theta_e = vars.θe_obs

    # ------------------------------------------------------------
    # Controller states
    # ------------------------------------------------------------

    ui_d = vars.ui_d
    ui_q = vars.ui_q

    # ------------------------------------------------------------
    # Controller algebraic variables
    # ------------------------------------------------------------

    isd_ref_lim = vars.isd_ref_lim
    isq_ref_lim = vars.isq_ref_lim

    isq_max_available = vars.isq_max_available

    err_d = vars.err_d
    err_q = vars.err_q

    vsd_PI = vars.vsd_PI
    vsq_PI = vars.vsq_PI

    vsd_ff = vars.vsd_ff
    vsq_ff = vars.vsq_ff

    vsd_unsat = vars.vsd_unsat
    vsq_unsat = vars.vsq_unsat

    vs_mod_unsat = vars.vs_mod_unsat
    voltage_scale = vars.voltage_scale
    voltage_saturation = vars.voltage_saturation

    vsd_ref = vars.vsd_ref
    vsq_ref = vars.vsq_ref

    is_mod_obs = vars.is_mod_obs

    vsα = vars.vsα
    vsβ = vars.vsβ

    # ------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------

    Rs = pars.Rs
    Lss = pars.Lss
    Lrr = pars.Lrr
    Lm = pars.Lm

    current_loop_tau = pars.current_loop_tau
    Vs_max = pars.Vs_max
    Is_max = pars.Is_max

    sigma_Lss = pars.sigma_Lss
    k_coupling = Lm / Lrr

    Kp = sigma_Lss / current_loop_tau
    Ki = Rs / current_loop_tau
    Kaw = 1 / Kp

    return [
        # --------------------------------------------------------
        # Current reference limiting with d-axis priority
        # --------------------------------------------------------
        isd_ref_lim ~ ifelse(
            abs(isd_ref) > Is_max,
            sign(isd_ref) * Is_max,
            isd_ref,
        ),

        isq_max_available ~ sqrt(max(Is_max^2 - isd_ref_lim^2, 0.0)),

        isq_ref_lim ~ ifelse(
            abs(isq_ref) > isq_max_available,
            sign(isq_ref) * isq_max_available,
            isq_ref,
        ),

        # --------------------------------------------------------
        # Current errors
        # --------------------------------------------------------
        err_d ~ isd_ref_lim - isd_med,
        err_q ~ isq_ref_lim - isq_med,

        # --------------------------------------------------------
        # PI current controller
        # --------------------------------------------------------
        vsd_PI ~ Kp * err_d + ui_d,
        vsq_PI ~ Kp * err_q + ui_q,

        # --------------------------------------------------------
        # Feedforward decoupling / back-EMF terms
        #
        # MATLAB:
        # vsd_ff = -omega_e * sigma_Lss * isq_filt
        # vsq_ff =  omega_e * sigma_Lss * isd_filt + omega_e * k * lambda_rd
        # --------------------------------------------------------
        vsd_ff ~ -omega_e * sigma_Lss * isq_med,
        vsq_ff ~  omega_e * sigma_Lss * isd_med + omega_e * k_coupling * lambda_rd,

        # --------------------------------------------------------
        # Unsaturated voltage command
        # --------------------------------------------------------
        vsd_unsat ~ vsd_PI + vsd_ff,
        vsq_unsat ~ vsq_PI + vsq_ff,

        vs_mod_unsat ~ sqrt(vsd_unsat^2 + vsq_unsat^2),

        # --------------------------------------------------------
        # Voltage vector saturation
        # --------------------------------------------------------
        voltage_scale ~ ifelse(
            vs_mod_unsat > Vs_max,
            Vs_max / (vs_mod_unsat + 1e-9),
            1.0,
        ),

        voltage_saturation ~ ifelse(vs_mod_unsat > Vs_max, 1.0, 0.0),

        vsd_ref ~ voltage_scale * vsd_unsat,
        vsq_ref ~ voltage_scale * vsq_unsat,

        # --------------------------------------------------------
        # Continuous-time anti-windup
        #
        # Back-calculation:
        # d(ui)/dt = Ki*err - Kaw*(v_unsat - v_sat)
        # --------------------------------------------------------
        D(ui_d) ~ Ki * err_d - Kaw * (vsd_unsat - vsd_ref),
        D(ui_q) ~ Ki * err_q - Kaw * (vsq_unsat - vsq_ref),

        # --------------------------------------------------------
        # Diagnostic current magnitude
        # --------------------------------------------------------
        is_mod_obs ~ sqrt(isd_med^2 + isq_med^2),

        # --------------------------------------------------------
        # Inverse Park transform: dq_e voltage -> stationary alpha-beta
        #
        # If:
        # id =  cos(theta)*iα + sin(theta)*iβ
        # iq = -sin(theta)*iα + cos(theta)*iβ
        #
        # then inverse is:
        # vα = cos(theta)*vd - sin(theta)*vq
        # vβ = sin(theta)*vd + cos(theta)*vq
        # --------------------------------------------------------
        vsα ~ cos(theta_e) * vsd_ref - sin(theta_e) * vsq_ref,
        vsβ ~ sin(theta_e) * vsd_ref + cos(theta_e) * vsq_ref,
    ]
end