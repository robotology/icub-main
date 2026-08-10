# embObjMotionControl Configuration Defaults (User Guide)

This document summarizes the default values used by embObjMotionControl when optional parameters are missing in configuration files.

Scope:
- Fallback values defined in the defaults singleton.
- Fallback behavior implemented by the parser.
- Practical notes for writing robust config files.

## 1. How defaults are applied

- If a parameter/group is optional and missing, embObjMotionControl uses internal fallback values.
- If a parameter/group is mandatory and missing, parsing fails.
- Some defaults are conditional (for example based on board type or encoder type).

## 2. Allowed string tokens (validation values)

These are accepted values for specific string parameters:

- `NONE`
- `low_lev_current`
- `direct`
- `minjerk`
- `torque`
- `position_metric`
- `velocity_metric`
- `torque_metric`
- `currnet_metric`
- `machine_units`
- `dutycycle_percent`
- `PT100`
- `PT1000`
- `prismatic`
- `revolute`

## 2.1 PID output type
   {"n_a",      {eomc_ctrl_out_type_n_a,  "Not applicable / Not set"}},
    {"pwm",      {eomc_ctrl_out_type_pwm,  "PWM (Pulse Width Modulation)"}},
    {"velocity", {eomc_ctrl_out_type_vel,  "Velocity control output"}},
    {"current",  {eomc_ctrl_out_type_cur,  "Current control output"}}

## 3. Defaults by configuration group

## 3.1 FOC group

Optional parameters and fallbacks:

- `TemperatureSensorType`: `NONE`
- `Verbose`: `0` (disabled)
- `AutoCalibration`: `0` (disabled)
- `Kbemf`: `0.0`

Notes:
- `HasTempSensor` is deprecated. Use `TemperatureSensorType`.
- If `TemperatureSensorType` is omitted, firmware temperature checks are disabled.

## 3.2 TIMEOUTS group

If `TIMEOUTS` is missing, these defaults are applied to all joints:

- `velocity`: 100 ms
- `current`: 100 ms
- `pwm`: 100 ms
- `torque`: 100 ms
- `torque_measure`: 100 ms

If only one timeout entry is missing, only that entry falls back to default.

AMC FOC safeguard:
- For advanced FOC service boards, each timeout is clamped to a minimum of 300 ms.
- Effective minimums are:
  - `velocity`: 300 ms
  - `current`: 300 ms
  - `pwm`: 300 ms
  - `torque`: 300 ms
  - `torque_measure`: 300 ms

## 3.3 LIMITS group (temperature)

Optional parameters and fallbacks:

- `hardwareTemperatureLimits`: 0.0
- `warningTemperatureLimits`: 0.0

Validation rule:
- `warningTemperatureLimits` must be less than or equal to 85% of `hardwareTemperatureLimits`.

## 3.4 Torque PID optional terms

When torque control is enabled, these torque compensation terms are optional and default to 0.0 if omitted:

- `kbemf`
- `viscousPos`
- `viscousNeg`
- `coulombPos`
- `coulombNeg`
- `velocityThres`

Mandatory (no fallback):
- `ktau`
- `filterType`
- standard PID terms required by the selected control law.

## 3.5 Position-direct and velocity-direct PID blocks

If `CONTROLS.positionDirect` or `CONTROLS.velocityDirect` is set to `NONE`, the corresponding PID is disabled.

Fallback behavior:
- Position direct PID: disabled.
- Velocity direct PID: disabled.
- Stored PID values are treated as inactive for those control modes.

## 3.6 KALMAN_FILTER group

If the whole group is missing, Kalman filtering is disabled for all joints and these defaults are used:

- `kalmanFilterEnabled`: `false`
- `x0`: `[0.0, 0.0, 0.0]`
- `Q`: `[0.0, 0.0, 0.0]`
- `R`: `0.0`
- `P0`: `0.0`

If the group exists, all listed fields are expected.

## 3.7 OTHER_CONTROL_PARAMETERS.deadZone

If `OTHER_CONTROL_PARAMETERS` or `deadZone` is missing, deadzone is set by joint encoder type:

- AEA encoder: 0.0494
- AEA3 encoder: 0.0
- AKSIM2 encoder: 0.00068
- AMO encoder: 0.0055
- Other encoders: intended fallback is 0.00068

## 3.8 LUGRE group

If the `LUGRE` group is missing:

- `Km` defaults to `-1.0`.
- Other LuGre parameters are not explicitly assigned by this fallback path.

If the group is present, all required LuGre fields must be provided.

## 4. Additional internal defaults (runtime behavior)

These are not direct parser fallbacks for configuration fields, but they affect runtime behavior:

- `jointPosLimitsDelta = 2.0`
  - Returned position limits are reduced by this margin in limit reporting logic.
- `maxPositionMoveInterval = 0.080 s`
  - If positionMove commands arrive faster than this threshold, a performance warning is issued.

## 5. Quick checklist for configuration authors

- Always provide all mandatory groups first (`LIMITS`, `CONTROLS`, joint and motor definitions, PID mandatory parts).
- If you omit optional fields, verify that defaults are compatible with your hardware.
- For AMC advanced FOC boards, remember effective timeout minimum is 300 ms.
- Explicitly set `TemperatureSensorType` and temperature limits if thermal protection is required.
- If you use deadzone defaults, verify encoder type mapping is correct for each joint.
