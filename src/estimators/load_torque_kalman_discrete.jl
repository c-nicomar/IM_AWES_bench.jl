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

"""
    LoadTorqueKalmanState(; kwargs...)

Mutable state of the load-torque Kalman estimator, advanced in place by
[`load_torque_kalman_step!`](@ref): the two-element state vector
`x = [omega_hat, TL_hat]` plus its 2×2 covariance, stored element by element to
keep the block allocation-free.

# Fields

- `omega_hat`: estimated mechanical speed in rad/s. Also the measured quantity,
  so this one is well determined.
- `TL_hat`: estimated load torque in N·m, positive when it accelerates toward
  positive speed (`J*dω/dt = Te + TL - B*ω`). This is the quantity of interest —
  it is never measured, only inferred from how the speed deviates from what the
  applied torque alone would produce.
- `P11`, `P12`, `P21`, `P22`: error covariance entries. `P22` starts large
  (`100.0`) because the initial load torque is genuinely unknown, which lets the
  filter converge quickly at startup.
- `initialized`: `false` until the first step. The first call therefore
  self-initialises, seeding `omega_hat` with the measured speed even if `reset`
  was not passed.

See also [`LoadTorqueKalmanParams`](@ref), [`LoadTorqueKalmanOutput`](@ref).
"""
Base.@kwdef mutable struct LoadTorqueKalmanState
    omega_hat::Float64 = 0.0
    TL_hat::Float64 = 0.0

    P11::Float64 = 1.0
    P12::Float64 = 0.0
    P21::Float64 = 0.0
    P22::Float64 = 100.0

    initialized::Bool = false
end

"""
    LoadTorqueKalmanParams(; kwargs...)

Model and tuning of the load-torque Kalman estimator, consumed by
[`load_torque_kalman_step!`](@ref).

`J` and `B` define the mechanical model the filter inverts, so an error in them
shows up directly as a bias in `TL_hat`. The three noise variances are the only
real tuning knobs: the ratio `q_TL/R` sets how fast the estimate tracks a
changing load versus how much measurement noise it lets through.

# Fields

- `J`: total inertia in kg·m², motor plus load.
- `B`: viscous friction coefficient in N·m·s/rad.
- `Ts`: sample time in s.
- `R`: speed measurement noise variance in (rad/s)². Raise it for a noisy
  encoder; non-positive values fall back to `0.01`.
- `q_omega`: process noise variance on the speed state. Absorbs unmodelled
  mechanical effects; non-positive values fall back to `0.1`.
- `q_TL`: process noise variance on the load-torque state. The load is modelled
  as a random walk (`TL[k+1] = TL[k]`), so this is what allows it to move at
  all — larger means faster tracking and more noise. Non-positive values fall
  back to `6.0`.
- `limit_positive`: when `true`, clamp `TL_hat` at zero, as the MATLAB original
  did for a load that could only brake. **Set this to `false` for AWES
  profiles**, where the tether torque genuinely changes sign between reel-out
  and reel-in; leaving it `true` would silently discard half the signal.

See also [`LoadTorqueKalmanState`](@ref), [`LoadTorqueKalmanOutput`](@ref).
"""
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

"""
    LoadTorqueKalmanOutput(; kwargs...)

Result of one [`load_torque_kalman_step!`](@ref) call, returned fresh each
sample.

# Fields

- `TL_hat`: estimated load torque in N·m, positive when it accelerates toward
  positive speed. This is the value to feed to a speed loop's `TL_est`.
- `omega_hat`: estimated mechanical speed in rad/s after the measurement update.
- `innovation`: measurement residual in rad/s, `omega - omega_pred`. The natural
  health check — it should look like zero-mean noise. A persistent offset means
  `J`, `B` or `Ts` do not match the real mechanics.
- `K1`, `K2`: the two Kalman gains applied to the innovation this sample. `K2`
  is the one that drives `TL_hat`; watching it settle shows the filter
  converging.

See also [`LoadTorqueKalmanParams`](@ref).
"""
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

"""
    load_torque_kalman_step!(st, p; omega, Te, reset = false)

Advance the load-torque Kalman estimator by one sample period `p.Ts` and return
the current estimate of the unmeasured load torque.

Julia counterpart of the MATLAB function `l3_kalman`. The load torque is treated
as a second state driven by a random walk, so it can be reconstructed from the
speed measurement and the known applied torque:

```
ω[k+1]  = (1 - B*Ts/J)*ω[k] + (Ts/J)*TL[k] + (Ts/J)*Te[k]
TL[k+1] = TL[k]
y       = ω
```

which is the discrete Euler form of `J*dω/dt = Te + TL - B*ω`. Note the sign
that follows from it: the coupling from `TL` into the speed state is `+Ts/J`,
not `-Ts/J`, because a positive load torque accelerates toward positive speed
in this convention. Getting that backwards makes the estimate converge to the
negative of the true load, which then silently reverses the feedforward that
consumes it.

`st` is mutated in place; the result is a fresh
[`LoadTorqueKalmanOutput`](@ref).

# Arguments

- `st::LoadTorqueKalmanState`: state vector and covariance, mutated.
- `p::LoadTorqueKalmanParams`: mechanical model and noise tuning.
- `omega`: measured mechanical speed in rad/s.
- `Te`: applied electromagnetic torque in N·m — the known input. In the hybrid
  loop this is the observer's torque estimate, not a measurement.
- `reset`: when `true`, reinitialise the filter with `omega_hat = omega`,
  `TL_hat = 0` and the default covariance. This also happens automatically on
  the first call, since `st.initialized` starts `false`.

# Steps

1. **Predict** the state through the model above and propagate the covariance,
   adding `q_omega` and `q_TL`.
2. **Update** with the speed measurement: `S = P11 + R`, gains `K1 = P11/S` and
   `K2 = P21/S`, correction by `innovation = omega - omega_pred`.
3. **Clamp**, if `p.limit_positive`, `TL_hat` at zero. Leave this off for signed
   AWES torque profiles.
4. **Symmetrise** the covariance, averaging `P12` and `P21`, to keep it from
   drifting apart numerically over long runs.

Both speed outer loops accept the result as their `TL_est` argument — see
[`outer_speed_flux_f1_step!`](@ref) and [`outer_speed_flux_mtpa_step!`](@ref).
Feedforward only engages when their `use_load_feedforward` is `true`.
"""
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