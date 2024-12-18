![YARP logo](https://raw.githubusercontent.com/robotology/yarp/master/doc/images/yarp-robot-24.png "couplingICubEye")
## couplingICubEye


This is the device for [YARP](https://www.yarp.it/) for handling the coupling of the [eyes](https://icub-tech-iit.github.io/documentation/icub_kinematics/icub-vergence-version/icub-vergence-version/)

## Device documentation

This device driver exposes the `yarp::dev::IJointCoupling` interface to getting
quantities from the physical joints to the actuated axes domain and viceversa.
See the documentation for more details about each interface.


| YARP device name |
|:----------------:|
| `couplingICubEye`     |

Parameters used by this device are:

| Parameter name               | SubParameter      | Type           | Read / write | Units   | Default Value | Required        | Description                                                                           | Notes                                                                 |
|:----------------------------:|:-----------------:|:--------------:|:------------:|:-------:|:-------------:|:---------------:|:-------------------------------------------------------------------------------------:|:---------------------------------------------------------------------:|
|  `jointNames`                      |     -             |   list of strings       | Read / write | -       |   -           |  Yes            | Names of the physical joints                                   |                                   |
|  `LIMITS`                      |     -             |   group      | Read / write | -       |   -           |  Yes            | group containing the physical joint limits                                     |                                   |
|                      |     `jntPosMin`        | list of double     | Read / write | -       |   -           |  Yes            | Physical joints' position minimum                                  |                      |
|                      |     `jntPosMax`        | list of double     | Read / write | -       |   -           |  Yes            | Physical joints' position maximum                                  |                                 |
|  `COUPLING`                      |     -             | group         | Read / write | -       |   -           |  Yes            | The group containing the coupling description
|                      |     `actuatedAxesNames`        | list of strings     | Read / write | -       |   -           |  Yes            | Names of the actuated axes                                      |                                   |
|                      |     `actuatedAxesPosMin`        | list of strings     | Read / write | -       |   -           |  Yes            | Actuated axes' position minimum                                      |                                   |
|                      |     `actuatedAxesPosMax`        | list of strings     | Read / write | -       |   -           |  Yes            | Actuated axes' position maximum                                      |                                   |


Configuration file using `.ini` format:

```ini
device              couplingICubEye
jointNames          eyes_tilt l_eye_pan_joint r_eye_pan_joint

[COUPLING]
device              couplingICubEye 
actuatedAxesNames   eyes_tilt eyes_version eyes_vergence
actuatedAxesPosMin  -30.0     -30.0   0.0
actuatedAxesPosMax  30.0      30.0    50.0

[LIMITS]
jntPosMax           30.0    55.0    30.0
jntPosMin           -30.0   -30.0   -55.0
jntVelMax           100.0   100.0   100.0
```