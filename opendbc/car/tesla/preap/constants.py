# Pre-AP pedal accel envelopes, mapped to openpilot Driving Personality toggle.
# Breakpoints are speed in m/s; values are max accel in m/s².
# Based on Tinkla Pedal profiles but reduced for feedforward architecture:
# Tinkla uses PID which naturally dampens the ramp to the ceiling; our kf=1.0
# feedforward passes MPC targets straight through, so the ceilings must be
# lower to get the same feel.
#   aggressive(0) → spirited but controlled
#   standard(1)   → smooth daily driver
#   relaxed(2)    → gentle launch with gradual speed recovery
# Extra speed points shape the Relaxed response above 20 mph. The other
# profiles retain their previous piecewise-linear envelopes.
ACCEL_PREAP_BP = [0.0, 1.3, 7.5, 8.9408, 11.176, 13.4112, 15.0, 17.8816, 22.352, 25.0, 26.8224, 31.2928, 40.0]  # m/s
ACCEL_PREAP_PROFILES = {
  0: [0.4, 0.8, 1.1, 1.081, 1.051, 1.021, 1, 0.957, 0.89, 0.85, 0.832, 0.787, 0.7],  # aggressive
  1: [0.3, 0.7, 1, 0.981, 0.951, 0.921, 0.9, 0.871, 0.826, 0.8, 0.782, 0.737, 0.65],  # standard
  2: [0.14, 0.24, 0.34, 0.35, 0.35, 0.38, 0.42, 0.50, 0.56, 0.54, 0.52, 0.50, 0.47],  # relaxed: gentle launch, smooth pickup
}

# When following a lead car, cap positive accel to these values regardless
# of personality. Prevents overshoot → regen → overshoot oscillation.
# Open road uses the full profile above; this only limits follow mode.
# PREAP_MPC_LEAD_CAP_V1
ACCEL_PREAP_FOLLOW = [0.9, 1.1, 0.9, 0.842, 0.753, 0.664, 0.6, 0.557, 0.49, 0.45, 0.438, 0.408, 0.35]

# Feedforward-dominant longitudinal tune (FrogPilot/OPGM Bolt-inspired).
# Modest speed-dependent kp provides immediate correction when actual
# acceleration differs from the plan. It remains low at walking speed
# to avoid amplifying aEgo noise.
# kf=1.0: full a_target passthrough — the MPC plan (jerk-constrained, smooth)
#   is the dominant control signal.
# ki low: slow integral trim handles steady-state offset (hills, wind, regen).
#   Lower than FP Bolt (0.125-0.33) because our kf=1.0 vs their kf=0.25.
PEDAL_LONG_K_BP = [0.0, 3.0, 6.0, 35.0]
# PREAP_SINGLE_LOOP_RECOVERY_V1
PEDAL_LONG_KP_V = [0.0, 0.0, 0.0, 0.0]  # aEgo noise must not directly move the pedal
PEDAL_LONG_KI_V = [0.04, 0.06, 0.10, 0.15]  # Slow trim for hills, drag, and steady speed

# Virtual DAS inner PID — sits inside carcontroller, corrects the
# feedforward output by tracking (a_cmd - a_ego_future). Conservative
# initial tune: no kp (same philosophy as outer loop), ki faster than
# outer so inner loop settles before outer reacts.
VDAS_INNER_K_BP = [0.0, 5.0, 35.0]
VDAS_INNER_KP_V = [0.0, 0.0, 0.0]
VDAS_INNER_KI_V = [0.0, 0.0, 0.0]  # Disabled: LongControl is the sole feedback loop

# Delay compensation: predict a_ego this far into the future using
# estimated jerk. Longer at highway speed where powertrain is slower.
VDAS_FUTURE_T_BP = [2.0, 5.0]
VDAS_FUTURE_T_V = [0.30, 0.55]

# a_ego low-pass filter time constant (seconds). Smooths IMU noise
# without adding too much phase lag. Matches Toyota's 0.25s RC.
VDAS_AEGO_FILTER_RC = 0.25
