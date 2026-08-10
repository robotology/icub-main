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

All tokens are defined as `constexpr std::string_view` in `namespace yarp::dev::eomc::ParamValues` inside `embObjMotionControlDefaults.h`.

### 2.1 Control law (`ParamValues::controlLaw`)

| Human-readable name   | Config value        | C++ symbol                           |
|-----------------------|---------------------|--------------------------------------|
| Low-level current     | `low_lev_current`   | `ParamValues::controlLaw::current`   |
| Direct                | `direct`            | `ParamValues::controlLaw::direct`    |
| Minimum jerk          | `minjerk`           | `ParamValues::controlLaw::trajectory`|
| Torque                | `torque`            | `ParamValues::controlLaw::torque`    |

### 2.2 Feedback control units (`ParamValues::fbkUnits`)

| Human-readable name   | Config value    | C++ symbol                        |
|-----------------------|-----------------|-----------------------------------|
| Metric units          | `metric_units`  | `ParamValues::fbkUnits::metric`   |
| Machine units         | `machine_units` | `ParamValues::fbkUnits::machine`  |

### 2.3 Output control units (`ParamValues::outUnits`)

| Human-readable name      | Config value         | C++ symbol                          |
|--------------------------|----------------------|-------------------------------------|
| Position metric          | `position_metric`    | `ParamValues::outUnits::pos_metric` |
| Velocity metric          | `velocity_metric`    | `ParamValues::outUnits::vel_metric` |
| Torque metric            | `torque_metric`      | `ParamValues::outUnits::trq_metric` |
| Current metric           | `current_metric`     | `ParamValues::outUnits::cur_metric` |
| Duty cycle (percent)     | `dutycycle_percent`  | `ParamValues::outUnits::dutycycle`  |
| Machine units            | `machine_units`      | `ParamValues::outUnits::machine`    |

### 2.4 Output type (`ParamValues::outputType`)

| Human-readable name      | Config value | C++ symbol                          |
|--------------------------|--------------|-------------------------------------|
| Not applicable           | `n_a`        | `ParamValues::outputType::n_a`      |
| PWM                      | `pwm`        | `ParamValues::outputType::pwm`      |
| Velocity                 | `velocity`   | `ParamValues::outputType::velocity` |
| Current                  | `current`    | `ParamValues::outputType::current`  |

### 2.5 Temperature sensor type (`ParamValues::temperatureSensor`)

| Human-readable name | Config value | C++ symbol                               |
|---------------------|--------------|------------------------------------------|
| None                | `NONE`       | `ParamValues::temperatureSensor::none`   |
| PT100 sensor        | `PT100`      | `ParamValues::temperatureSensor::PT100`  |
| PT1000 sensor       | `PT1000`     | `ParamValues::temperatureSensor::PT1000` |

### 2.6 Axis type (`ParamValues::axisType`)

| Human-readable name | Config value | C++ symbol                          |
|---------------------|--------------|-------------------------------------|
| Prismatic joint     | `prismatic`  | `ParamValues::axisType::prismatic`  |
| Revolute joint      | `revolute`   | `ParamValues::axisType::revolute`   |

### 2.7 Control mode keys (`ParamValues::controlMode`)

These are the keys used in the `CONTROLS` config group to associate each control mode with a named PID group.

| Human-readable name     | Config key        | C++ symbol                               |
|-------------------------|-------------------|------------------------------------------|
| Position control        | `positionControl` | `ParamValues::controlMode::position`     |
| Velocity control        | `velocityControl` | `ParamValues::controlMode::velocity`     |
| Mixed control           | `mixedControl`    | `ParamValues::controlMode::mixed`        |
| Torque control          | `torqueControl`   | `ParamValues::controlMode::torque`       |
| Current PID             | `currentPid`      | `ParamValues::controlMode::current`      |
| Position direct control | `positionDirect`  | `ParamValues::controlMode::positionDirect` |
| Velocity direct control | `velocityDirect`  | `ParamValues::controlMode::velocityDirect` |

## 3. Defaults by configuration group

## 3.1 FOC group

Optional parameters and fallbacks:

| Parameter              | Default value | C++ field                                        |
|------------------------|---------------|--------------------------------------------------|
| `TemperatureSensorType`| `NONE`        | `ParamValues::temperatureSensor::none`           |
| `Verbose`              | `0`           | `defaults.focDefaults.verbose`                   |
| `AutoCalibration`      | `0`           | `defaults.focDefaults.autoCalibration`           |
| `Kbemf`                | `0.0`         | `defaults.focDefaults.kbemf_foc`                 |

Notes:
- `HasTempSensor` is deprecated. Use `TemperatureSensorType`.
- If `TemperatureSensorType` is omitted, firmware temperature checks are disabled.

## 3.2 TIMEOUTS group

If `TIMEOUTS` is missing, these defaults are applied to all joints:

| Timeout parameter | Default (ms) | C++ field                                    |
|-------------------|-------------|----------------------------------------------|
| `velocity`        | 100         | `defaults.timeoutsDefaults.velocity_ref`     |
| `current`         | 100         | `defaults.timeoutsDefaults.current_ref`      |
| `pwm`             | 100         | `defaults.timeoutsDefaults.pwm_ref`          |
| `torque`          | 100         | `defaults.timeoutsDefaults.torque_ref`       |
| `torque_measure`  | 100         | `defaults.timeoutsDefaults.torque_fbk`       |

If only one timeout entry is missing, only that entry falls back to default.

AMC FOC safeguard — effective minimums for AMC boards:

| Timeout parameter | Minimum (ms) | C++ field                                        |
|-------------------|-------------|--------------------------------------------------|
| `velocity`        | 300         | `defaults.timeoutsDefaults_amcFoc.velocity_ref`  |
| `current`         | 300         | `defaults.timeoutsDefaults_amcFoc.current_ref`   |
| `pwm`             | 300         | `defaults.timeoutsDefaults_amcFoc.pwm_ref`       |
| `torque`          | 300         | `defaults.timeoutsDefaults_amcFoc.torque_ref`    |
| `torque_measure`  | 300         | `defaults.timeoutsDefaults_amcFoc.torque_fbk`    |

## 3.3 LIMITS group (temperature)

Optional parameters and fallbacks:

| Parameter                    | Default | C++ field                                              |
|------------------------------|---------|--------------------------------------------------------|
| `hardwareTemperatureLimits`  | 0.0     | `defaults.temperatureLimitsDefaults.hardware`          |
| `warningTemperatureLimits`   | 0.0     | `defaults.temperatureLimitsDefaults.warning`           |

Validation rule:
- `warningTemperatureLimits` must be less than or equal to 85% of `hardwareTemperatureLimits`.

## 3.4 Torque PID optional terms

When torque control is enabled, these torque compensation terms are optional:

| Parameter       | Default | C++ field                                         |
|-----------------|---------|---------------------------------------------------|
| `kbemf`         | 0.0     | `defaults.torquePidDefaults.kbemf`                |
| `viscousPos`    | 0.0     | `defaults.torquePidDefaults.viscousPos`           |
| `viscousNeg`    | 0.0     | `defaults.torquePidDefaults.viscousNeg`           |
| `coulombPos`    | 0.0     | `defaults.torquePidDefaults.coulombPos`           |
| `coulombNeg`    | 0.0     | `defaults.torquePidDefaults.coulombNeg`           |
| `velocityThres` | 0.0     | `defaults.torquePidDefaults.velocityThres`        |

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

If the whole group is missing, Kalman filtering is disabled for all joints:

| Parameter             | Default         | C++ field                                                            |
|-----------------------|-----------------|----------------------------------------------------------------------|
| `kalmanFilterEnabled` | `false`         | `defaults.controlParametersDefaults.kalmanFilterParams.enabled`      |
| `x0`                  | `[0.0, 0.0, 0.0]` | `defaults.controlParametersDefaults.kalmanFilterParams.x0`         |
| `Q`                   | `[0.0, 0.0, 0.0]` | `defaults.controlParametersDefaults.kalmanFilterParams.Q`          |
| `R`                   | `0.0`           | `defaults.controlParametersDefaults.kalmanFilterParams.R`            |
| `P0`                  | `0.0`           | `defaults.controlParametersDefaults.kalmanFilterParams.P0`           |

If the group exists, all listed fields are expected.

## 3.7 OTHER_CONTROL_PARAMETERS.deadZone

If `OTHER_CONTROL_PARAMETERS` or `deadZone` is missing, deadzone is set by joint encoder type:

| Encoder type  | Default value | C++ field                                                       |
|---------------|---------------|-----------------------------------------------------------------|
| AEA           | 0.0494        | `defaults.controlParametersDefaults.deadzone.jointWithAEA`     |
| AEA3          | 0.0           | `defaults.controlParametersDefaults.deadzone.jointWithAEA3`    |
| AMO           | 0.0055        | `defaults.controlParametersDefaults.deadzone.jointWithAMO`     |
| AKSIM2        | 0.00068       | `defaults.controlParametersDefaults.deadzone.jointWithAKSIM2`  |
| Other         | 0.00068       | `defaults.controlParametersDefaults.deadzone.otherEncoder`     |

## 3.8 LUGRE group

If the `LUGRE` group is missing:

| Parameter | Default | C++ field                                         |
|-----------|---------|---------------------------------------------------|
| `Km`      | -1.0    | `defaults.controlParametersDefaults.lugre.Km`    |

Other LuGre parameters are not assigned by this fallback path; the group must be fully present if used.

## 4. Additional internal defaults (runtime behavior)

| Parameter                | Default   | C++ field                           | Notes                                                                 |
|--------------------------|-----------|-------------------------------------|-----------------------------------------------------------------------|
| `jointPosLimitsDelta`    | 2.0 °     | `defaults.jointPosLimitsDelta`      | Position limits are reduced by this margin in limit reporting logic.  |
| `maxPositionMoveInterval`| 0.080 s   | `defaults.maxPositionMoveInterval`  | positionMove commands arriving faster trigger a performance warning.  |

## 5. Parser constraints (firmware and logic)

These constraints are enforced by the parser at startup. Violating any of them causes a fatal error and prevents the device from opening.

### 5.1 Joint-set homogeneity (`areControlPidGroupEqualInJointSet`)

All joints that belong to the same joint set **must reference the same PID group name** for each control mode. This is required because the firmware applies a single set of PID parameters per joint set, not per individual joint.

| Checked in           | Constraint                                                              |
|----------------------|-------------------------------------------------------------------------|
| All control modes    | Joints in the same set must share the same `CONTROLS.<mode>` group name |

### 5.2 Shared PID name across position / velocity / mixed control

The embedded firmware uses **one PID block** for position, velocity and mixed control simultaneously. Therefore:

| Constraint                              | Error if violated                                                                    |
|-----------------------------------------|--------------------------------------------------------------------------------------|
| `velocityControl` name == `positionControl` name (or `NONE`) | `velocityControl` must equal `positionControl` or be `NONE` for each joint |
| `mixedControl` name == `positionControl` name (or `NONE`)    | `mixedControl` must equal `positionControl` or be `NONE` for each joint    |

### 5.3 Mandatory control law per PID type

Each PID group must declare the correct `controlLaw` token or parsing fails.

| PID type              | Required `controlLaw`                        | `ParamValues` symbol                  |
|-----------------------|----------------------------------------------|---------------------------------------|
| Current PID           | `low_lev_current`                            | `ParamValues::controlLaw::current`    |
| Position control      | `minjerk`                                    | `ParamValues::controlLaw::trajectory` |
| Velocity control      | `minjerk`                                    | `ParamValues::controlLaw::trajectory` |
| Mixed control         | `minjerk`                                    | `ParamValues::controlLaw::trajectory` |
| Position direct       | `direct`                                     | `ParamValues::controlLaw::direct`     |
| Velocity direct       | `direct`                                     | `ParamValues::controlLaw::direct`     |
| Torque control        | `torque`                                     | `ParamValues::controlLaw::torque`     |

### 5.4 Mandatory PID groups

| PID type         | Mandatory?                              | Notes                                                    |
|------------------|-----------------------------------------|----------------------------------------------------------|
| Position control | Yes — at least one joint must be active | Parsing fails if all joints have `positionControl=NONE`  |
| Current PID      | Conditional (`lowLevPidisMandatory`)    | Depends on board/service type; at least one joint active |
| Velocity control | No                                      | Warning logged if all disabled                           |
| Mixed control    | No                                      | Informational log if all disabled                        |
| Position direct  | No                                      | Treated as all-zero PID if absent                        |
| Velocity direct  | No                                      | Treated as all-zero PID if absent                        |
| Torque control   | No                                      | Silently disabled if absent                              |

### 5.5 Rotor encoder index consistency

| Constraint                                                        | Check location        |
|-------------------------------------------------------------------|-----------------------|
| `RotorIndexOffset` must be in `[0, 359]`                          | `parseFocGroup`       |
| If `HasRotorEncoderIndex == 0`, then `RotorIndexOffset` must be 0 | `parseFocGroup`       |
| `HasTempSensor` is **deprecated** — using it is a fatal error     | `parseFocGroup`       |

### 5.6 Mechanical parameter constraints

| Parameter          | Constraint                        | Error if violated                                         |
|--------------------|-----------------------------------|-----------------------------------------------------------|
| `Gearbox_M2J`      | Must not be 0 for any joint       | Division by zero risk in transmission math                |
| `Gearbox_E2J`      | Must not be 0 for any joint       | Division by zero risk in transmission math                |
| `motorPwmLimit`    | Must be ≥ 0                       | Negative PWM limit is physically meaningless              |
| `fullscalePWM`     | Must be > 0                       | Used as divisor in duty-cycle conversion                  |
| `ampsToSensor`     | Must be > 0                       | Used as divisor in current conversion                     |

### 5.7 Joint position limit consistency

| Constraint                                          | Config parameters involved              |
|-----------------------------------------------------|-----------------------------------------|
| `jntPosMax` ≤ `hardwareJntPosMax` for every joint  | `LIMITS.jntPosMax`, `hardwareJntPosMax` |
| `jntPosMin` ≥ `hardwareJntPosMin` for every joint  | `LIMITS.jntPosMin`, `hardwareJntPosMin` |

### 5.8 Temperature limit consistency

| Constraint                                                           | Threshold |
|----------------------------------------------------------------------|-----------|
| `warningTemperatureLimits` < 85 % of `hardwareTemperatureLimits`    | 0.85 ×    |

### 5.9 Swap direct velocity output
Currently the hot-swap of output type of velocity direct (pwm into current and viceversa) is't **unsupported**
s
### 5.10 AxisMap and AxisType validation

| Constraint                                            | Notes                                    |
|-------------------------------------------------------|------------------------------------------|
| Every `AxisMap` index must be < total number of joints | Out-of-range indices cause a fatal error |
| `AxisType` must be `revolute` or `prismatic`           | Unknown values cause a fatal error       |

Currently this vales aren't used by Embedded Motion Control


## 6. Quick checklist for configuration authors

- Always provide all mandatory groups first (`LIMITS`, `CONTROLS`, joint and motor definitions, PID mandatory parts).
- If you omit optional fields, verify that defaults are compatible with your hardware.
- For AMC advanced FOC boards, remember effective timeout minimum is 300 ms.
- Explicitly set `TemperatureSensorType` and temperature limits if thermal protection is required.
- If you use deadzone defaults, verify encoder type mapping is correct for each joint.
- `velocityControl` and `mixedControl` must share the same PID group name as `positionControl` (or be `NONE`).
- Do not use `HasTempSensor` — it is deprecated and will cause a startup failure.
- Never set `Gearbox_M2J`, `Gearbox_E2J`, `fullscalePWM`, or `ampsToSensor` to zero.


