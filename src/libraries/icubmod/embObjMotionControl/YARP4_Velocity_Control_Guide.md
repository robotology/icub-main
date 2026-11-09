# YARP 4 Motor Control Modes and Velocity Control

This document describes the motor-control extensions introduced during the work related to [robotology/icub-firmware#615](https://github.com/robotology/icub-firmware/issues/615).

The main goals are:

- Introduce a dedicated **Velocity Direct** control mode.
- Allow velocity control to generate either:
  - a direct **PWM** command, or
  - a **current** reference processed by the internal current loop.
- Make the different combinations of high-level control modes and low-level outputs configurable.
- Update the YARP interfaces, configuration files, parser, communication protocol, EMS firmware, and 2FOC firmware accordingly.
- Provide clearer PID configuration and descriptions in `yarpmotorgui`.
- Introduce the `FeedFoward`parameter in the `Velocity Direct` with inner current control. 
- Configure Kbemf in as motor parameter and use it in current control loop.

---

## 1. Background

The original velocity-control implementation relied on a trajectory generator, commonly referred to as **MinJerk**. The input velocity reference was first processed by the trajectory generator before being passed to the subsequent control loops.

This implementation remains available and is referred to as the **standard** or **classic velocity control** mode.

The new architecture also supports direct control modes in which the trajectory generator is bypassed. This is useful when the user needs to send a velocity reference directly to the control loop without additional interpolation.

---

## 2. Control Architecture

The control chain can be divided into two main levels:

### High-level control

High-level controllers generally run at approximately 1 kHz and include:

- Position control.
- Velocity control.
- Mixed position/velocity control.
- Position Direct control.
- Velocity Direct control.
- The trajectory-generation stage, such as MinJerk.

### Low-level control

Low-level controllers are implemented on the motor-control boards and include:

- PWM output.
- Current control.
- Velocity control.
- Velocity control with an internal current loop.
- Current limiting and motor-side feedback processing.

The final control law is selected through the motor-control configuration files. Different control schemes can be obtained by enabling or disabling branches of the general control structure and by configuring the corresponding PID parameters.

---

## 3. Control Modes

### 3.1 Standard Position Control

In standard position control, the position reference is processed by the trajectory generator. The generated trajectory is then used by the position controller.

The output of the position controller can be connected to different low-level control modes, such as:

- PWM.
- Current.
- Velocity.
- Velocity with an internal current loop.

The trajectory generator is enabled in this mode.

---

### 3.2 Position Direct Control

Position Direct bypasses the trajectory generator.

The position reference is sent directly to the position controller. A velocity reference can also be sent together with the position reference and used as a feedforward term.

Position Direct can be combined with the following low-level outputs:

- PWM.
- Current.
- Velocity.
- Velocity with an internal current loop.

The position-direct PID is already available on the EMS board. The associated PID parameters must be correctly exposed through the configuration protocol and YARP configuration pipeline.

---

### 3.3 Standard Velocity Control

Standard velocity control uses the trajectory generator. The velocity reference is interpolated before being sent to the velocity controller.

The velocity controller can generate:

- A PWM command.
- A current reference.
- A velocity reference for a lower-level velocity loop.
- A velocity reference followed by the internal current loop.

This mode is intended to preserve compatibility with the existing control behavior.

---

### 3.4 Velocity Direct Control

**Velocity Direct** is the main new YARP 4 control mode.

In this mode:

- The velocity reference is sent directly to the velocity controller.
- The MinJerk trajectory generator is bypassed.
- The output can be configured as either:
  - PWM, or
  - current.

The mode is intended for applications requiring direct velocity control on the motor-side.

The new YARP interface exposes the direct velocity command through a dedicated interface containing the `setVelocity` operation. 

**In 2FOC case**: The user configures the PID gains in machine-unit and velocity references have to be in metric unit.

The Velocity Direct mode is explicitly distinguished from the original velocity-control mode.

---

### 3.5 Mixed Control

The Mixed mode accepts both position and velocity references.

The velocity reference can be used as a feedforward component while the position reference is processed by the position controller. The exact behavior depends on the motor-control configuration.

The existing YARP `MIXED` mode was considered for this purpose. However, the final control law must be defined explicitly at the configuration and firmware levels rather than relying only on the historical behavior of the interface.

---

## 4. Low-Level Output Modes

The high-level controller can be connected to different low-level output stages.

### 4.1 PWM Output

With PWM output, the controller directly generates a PWM command for the motor.

The current loop is not explicitly included in the configured control chain. However, the 2FOC board can still apply current limiting where required.

This mode is useful when the user needs direct torque-like actuation through PWM.

---

### 4.2 Current Output

With current output, the high-level controller generates a current reference.

The current reference is then processed by the internal current loop on the 2FOC board. This provides a cascaded architecture in which the current controller regulates the motor current take into account the `Kbemf` paramter

This mode is useful when current regulation and torque control are required.

---

### 4.3 Velocity Output

With velocity output, the high-level control generates a velocity reference for a lower-level velocity controller.

This allows the control law to be split across different levels, for example:

```text
Position controller → Velocity controller → PWM
```

or:

```text
Position controller → Velocity controller → Current controller
```

---

### 4.4 Velocity with Internal Current Loop

The `velocity_current` output combines a velocity loop and an internal current loop:

```text
Velocity controller → Current controller → Motor
```

The velocity controller generates a current reference, which is then processed by the current loop on the 2FOC board.



---

## 5. Motor Control Structure

The new architecture aims to provide a flexible control structure rather than implementing a separate fixed schema for every possible combination.


The following diagrams summarize the main control-mode combinations.

The diagrams are graphical representations of the control chains implemented across the EMS and 2FOC boards.

These schemes can be adapted to any other embedded board (for example, an AMC-family board), abstracting away where the control loop runs while preserving the control flow.

---

### 5.1 MinJerk Modes with PWM Output

This configuration includes the MinJerk trajectory generator and supports Position, Velocity, and Mixed high-level modes.

The final output is PWM.

![MinJerk Position/Velocity/Mixed with PWM output](https://github.com/user-attachments/assets/8197325a-d3a9-4781-9bc6-1dcc1e3c738c)

---

### 5.2 MinJerk Modes with Current Output

This configuration includes the MinJerk trajectory generator and generates a current reference for the internal current loop.

![MinJerk Position/Velocity/Mixed with current output](https://github.com/user-attachments/assets/ebc68d29-abfd-41e0-8ac0-c42141bc4016)

---

### 5.3 MinJerk Modes with Velocity Output

This configuration includes the MinJerk trajectory generator and generates a velocity reference for the lower-level velocity controller.

![MinJerk Position/Velocity/Mixed with velocity output](https://github.com/user-attachments/assets/d6d95760-b841-4e6c-9301-56007d9eadcb)

---

### 5.4 MinJerk Modes with Velocity and Current Control

This configuration includes:

```text
MinJerk → Position/Velocity/Mixed control → Velocity loop → Current loop
```

The `velocity_current` output mode is already available in the 2FOC board. The configuration protocol must expose and configure it consistently.

![MinJerk Position/Velocity/Mixed with velocity-current output](https://github.com/user-attachments/assets/438b58a9-476a-46c9-a0b3-cd18b571f730)

---

### 5.5 Position Direct with PWM Output

In this configuration, the MinJerk trajectory generator is bypassed.

The position reference is processed by the Position Direct PID, which generates a PWM command.

![Position Direct with PWM output](https://github.com/user-attachments/assets/f3529cf3-24f6-4bb9-a38f-e77ac050244d)

The Position Direct PID is already implemented on the EMS board. Its parameters must be made available through the configuration protocol.

---

### 5.6 Position Direct with Current Output

The Position Direct controller generates a current reference, which is processed by the internal current loop.

![Position Direct with current output](https://github.com/user-attachments/assets/76a640e8-1c56-4c78-95f0-45a2db2c4dc5)

The control chain is:

```text
Position reference → Position Direct PID → Current reference → Current loop
```

---

### 5.7 Position Direct with Velocity Output

The Position Direct controller generates a velocity reference for the lower-level velocity controller.

![Position Direct with velocity output](https://github.com/user-attachments/assets/29ec568d-8653-4cc2-b360-af7d580838ba)

The control chain is:

```text
Position reference → Position Direct PID → Velocity loop
```

A velocity feedforward reference can also be sent together with the position reference when supported by the selected interface and configuration.

---

### 5.8 Position Direct with Velocity and Current Control

This configuration combines Position Direct with a velocity loop and an internal current loop.

![Position Direct with velocity-current output](https://github.com/user-attachments/assets/0322dea1-dd41-4748-9654-a5077f59377c)

The control chain is:

```text
Position reference
        │
        ▼
Position Direct PID
        │
        ▼
Velocity loop
        │
        ▼
Current loop
```

Both the Position Direct PID and the `velocity_current` output mode must be configured through the complete software and firmware pipeline.

---

## 8. YARP 4 Changes

The integration of the new control modes requires YARP 4. 
Here is the link to the YARP4 change log: https://github.com/robotology/yarp/blob/master/doc/release/v4_0_0.MD

### Changes in YARP4 interfaces

#### Return Values

Several YARP interfaces changed their return type from `bool` to `ReturnValue`.

Implementations must use:

```cpp
ReturnValue_ok
```

instead of returning `true`.
#### IPIDControl interface

- Return Type Changes All PID interface methods now return _ReturnValue_ instead of bool 
- New 4 Methods Added:
  - `setPidFeedforwardRaw` - e'un valore satreeming che viene mandato e che viene moltiplicato per KFF sets the PID feedforward  [NOTE: in the protocol we use already the kff and offset. are thy used for the same purpose?]
  - `getPidOffsetRaw` - gets the PID offset (with double& instead of double_) 
  - `getPidFeedforwardRaw` - gets the PID feedforward 
  - `getPidExtraInfoRaw` and `getPidExtraInfosRaw` - get extra information about PIDs Parameter Changes 
  - `isPidEnabledRaw`: the enabled parameter changed from bool_ to bool& (reference instead of pointer)  [NOTE: no to be implemented]

#### IPositionControl

- Return Type Changes All PID interface methods now return _ReturnValue_ instead of bool 
- Some methos has been renamed to make the meanfully, but the implementation is the same 
>[!NOTE: ]
> - Methods containing `ref` refer to direct control references.
> - Methods containing `traj` refer to trajectory-based control references.

| # | Old Method Name (origin/master) | New Method Name (current) |
|---|---|---|
| 1 | `setRefSpeedRaw` | `setTrajSpeedRaw` |
| 2 | `setRefSpeedsRaw` | `setTrajSpeedsRaw` |
| 3 | `getRefSpeedRaw` | `getTrajSpeedRaw` |
| 4 | `getRefSpeedsRaw` | `getTrajSpeedsRaw` |
| 5 | `setRefAccelerationRaw` | `setTrajAccelerationRaw` |
| 6 | `setRefAccelerationsRaw` | `setTrajAccelerationsRaw` |
| 7 | `getRefAccelerationRaw` | `getTrajAccelerationRaw` |
| 8 | `getRefAccelerationsRaw` | `getTrajAccelerationsRaw` |

#### IVelocityControl

| # | Old Method Name (origin/master) | New Method Name (current) |
|---|---|---|
| 1 | `getRefVelocityRaw` | `getTargetVelocityRaw` |
| 2 | `getRefVelocitiesRaw` | `getTargetVelocitiesRaw` |

#### IVelocityDirect

YARP 4 introduces the `Velocity Direct` control mode.

This mode is distinct from the existing velocity mode:

| Control mode | Trajectory generator | Reference |
|---|---:|---|
| Velocity | Enabled | Velocity reference processed by the trajectory generator |
| Velocity Direct | Disabled | Direct velocity reference |

The direct mode is intended for applications where the velocity command must be applied without MinJerk interpolation.

These are the main functions

- `setRefVelocityRaw(int jnt, double vel)`
- `setRefVelocityRaw(const std::vector& vels)`
- `setRefVelocityRaw(const std::vector& jnts, const std::vector& vels)`
- `getRefVelocityRaw(const int jnt, double& vel)`
- `getRefVelocityRaw(std::vector& vels)`
- `getRefVelocityRaw(const std::vector& jnts, std::vector& vels)`

**Important Note:** The name `getRefVelocityRaw` has been **repurposed** for the new VelocityDirect interface, while in the IVelocityControl2 interface, it has been renamed to `getTargetVelocityRaw`.



#### PID Information and Descriptions

YARP 4 and `yarpmotorgui` support richer PID information.

A PID can include:

- PID type.
- Input type.
- Output type.
- Measurement units.
- Additional notes.
- A textual description.

A `PIDWithExtraInfo` object was introduced to expose additional PID information through setter and getter methods.

The GUI can display the PID description read from the XML configuration file.

Each control mode can be associated with multiple PID parameter sets. The configuration may contain separate PID sets for:

- Normal control.
- Direct control.
- Mixed control.

The presence of multiple PID sets does not necessarily mean that the control mode can be switched dynamically at runtime. The active set is selected according to the configured control mode.

#### MultipleAnalogSensor
the devices of type AnalofSensor have been replaced with MultipleAnalogSensor type





----------------

---------------------------

---

## 9. Configuration Changes

The motor-control XML configuration is extended to distinguish the different control modes and PID types.

The main configuration concepts include:

#### the MC file
 - Group CONTROL:
    - subtitute `speedPid` with `velocityDirect`
    - renamed `posDirectControl`in `positionDirect`
    - renamed `currentPid`in `currentControl`
    - the controlLaw of `positionDirect`in `direct`.
    - il control `velocityDirect`has `direct`in controlLaw instead of `lowlevelspeed`
    Note: all control types are mandatory

 - In each PID GROUP:
    - ControlLaw has value (see default)
    - `OutputType`is mandatory (see default values)

 #### The MEC file
 - bump `MotioncontrolVersion`to 7
 - Sobtituite `HasTempSensor`with `TemperatureSensorType`(see default values)

 ### Sensors configuration files
 Every sensors config type has been updated with MultipleAnalogSensor and the respective Wrapper and Remapper with MultipleAnalogServer. (AnalogServer & SkinWrapper ==> multipleanalogsensorsserver)
 
---




## References

- [Velocity control improvements – issue #615](https://github.com/robotology/icub-firmware/issues/615)
- [YARP pull request #3270](https://github.com/robotology/yarp/pull/3270)
- [Motor-control configuration examples](https://github.com/isorrentino/motor-control-configuration)
- [YARP documentation](https://www.yarp.it/)