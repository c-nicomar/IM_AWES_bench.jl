# ============================================================
# Discrete load torque Kalman estimator
# ============================================================
#
# Based on MATLAB function:
#
#   l3_kalman(omega, Te, Ts, mech, R, q_omega, q_TL, reset)
#
# Model:
#
#   TL + Te = J*dω/dt + B*ω
#
# equivalently:
#
#   J*dω/dt = Te + TL - B*ω
#
# Discrete Euler model:
#
#   ω[k+1]  = (1 - B*Ts/J)*ω[k] + (Ts/J)*TL[k] + (Ts/J)*Te[k]
#   TL[k+1] = TL[k]
# State:
#
#   x = [ω_hat, TL_hat]
#
# Measurement:
#
#   y = ω
# ============================================================

Base.@kwdef mutable struct LoadTorqueKalmanState
    omega_hat::Float64 = 0.0
    TL_hat::Float64 = 0.0

    P11::Float64 = 1.0
    P12::Float64 = 0.0
    P21::Float64 = 0.0
    P22::Float64 = 100.0

    initialized::Bool = false
end

Base.@kwdef struct LoadTorqueKalmanParams
    J::Float64 = 0.2685 + 0.1
    B::Float64 = 0.01298
    Ts::Float64 = 100e-6

    # Measurement noise variance [(rad/s)^2]
    R::Float64 = 0.01

    # Process noise variances
    q_omega::Float64 = 0.1
    q_TL::Float64 = 6.0

    # Enforce TL_hat >= 0, as in the MATLAB version
    limit_positive::Bool = true
end

Base.@kwdef struct LoadTorqueKalmanOutput
    TL_hat::Float64 = 0.0
    omega_hat::Float64 = 0.0

    innovation::Float64 = 0.0

    K1::Float64 = 0.0
    K2::Float64 = 0.0
end

function reset!(
    st::LoadTorqueKalmanState;
    omega0::Float64 = 0.0,
)
    st.omega_hat = omega0
    st.TL_hat = 0.0

    st.P11 = 1.0
    st.P12 = 0.0
    st.P21 = 0.0
    st.P22 = 100.0

    st.initialized = true

    return nothing
end

function load_torque_kalman_step!(
    st::LoadTorqueKalmanState,
    p::LoadTorqueKalmanParams;
    omega::Float64,
    Te::Float64,
    reset::Bool = false,
)

    if reset || !st.initialized
        reset!(st; omega0 = omega)
    end

    J = p.J
    B = p.B
    Ts = p.Ts

    R = p.R <= 0.0 ? 0.01 : p.R
    q_omega = p.q_omega <= 0.0 ? 0.1 : p.q_omega
    q_TL = p.q_TL <= 0.0 ? 6.0 : p.q_TL

    # ------------------------------------------------------------
    # Discrete model
    # ------------------------------------------------------------

    A11 = 1.0 - (B * Ts) / J
    A12 = Ts / J
    Bu1 = Ts / J

    Q11 = q_omega
    Q22 = q_TL

    # ------------------------------------------------------------
    # Prediction
    # ------------------------------------------------------------

    xpred_1 = A11 * st.omega_hat + A12 * st.TL_hat + Bu1 * Te
    xpred_2 = st.TL_hat

    Ppred_11 = A11 * A11 * st.P11 + 2.0 * A11 * A12 * st.P21 + A12 * A12 * st.P22 + Q11
    Ppred_12 = A11 * st.P12 + A12 * st.P22
    Ppred_21 = A11 * st.P21 + A12 * st.P22
    Ppred_22 = st.P22 + Q22

    # ------------------------------------------------------------
    # Measurement update
    # ------------------------------------------------------------

    S = Ppred_11 + R

    K1 = Ppred_11 / S
    K2 = Ppred_21 / S

    innovation = omega - xpred_1

    st.omega_hat = xpred_1 + K1 * innovation
    st.TL_hat = xpred_2 + K2 * innovation

    if p.limit_positive && st.TL_hat < 0.0
        st.TL_hat = 0.0
    end

    st.P11 = (1.0 - K1) * Ppred_11
    st.P12 = (1.0 - K1) * Ppred_12
    st.P21 = Ppred_21 - K2 * Ppred_11
    st.P22 = Ppred_22 - K2 * Ppred_12

    # Symmetrize covariance
    P12_sym = 0.5 * (st.P12 + st.P21)
    st.P12 = P12_sym
    st.P21 = P12_sym

    return LoadTorqueKalmanOutput(
        TL_hat = st.TL_hat,
        omega_hat = st.omega_hat,
        innovation = innovation,
        K1 = K1,
        K2 = K2,
    )
end