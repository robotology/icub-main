![YARP logo](https://raw.githubusercontent.com/robotology/yarp/master/doc/images/yarp-robot-24.png "yarp-device-realsense2")
## couplingICubHandMk2


This is the device for [YARP](https://www.yarp.it/) for handling the coupling of the hands mk2.

## Device documentation

This device driver exposes the `yarp::dev::IJointCoupling` interface to getting
quantities from the physical joints to the actuated axes domain and viceversa.
See the documentation for more details about each interface.


| YARP device name |
|:----------------:|
| `couplingICubHandMk2`     |

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
disableImplicitNetworkWrapper
yarpDeviceName left_hand_hardware_device

jointNames (l_hand_thumb_0_joint l_hand_thumb_1_joint l_hand_thumb_2_joint l_hand_thumb_3_joint l_hand_index_0_joint l_hand_index_1_joint l_hand_index_2_joint l_hand_index_3_joint l_hand_middle_0_joint l_hand_middle_1_joint l_hand_middle_2_joint l_hand_middle_3_joint l_hand_ring_0_joint l_hand_ring_1_joint l_hand_ring_2_joint l_hand_ring_3_joint l_hand_little_0_joint l_hand_little_1_joint l_hand_little_2_joint l_hand_little_3_joint)

min_stiffness (0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0)
max_stiffness (1000.0 1000.0 1000.0 1000.0 1000.0 1000.0 1000.0 1000.0 1000.0 1000.0 1000.0 1000.0 1000.0 1000.0 1000.0 1000.0 1000.0 1000.0 1000.0 1000.0)
min_damping   (0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0    0.0)
max_damping   (100.0  100.0  100.0  100.0  100.0  100.0  100.0  100.0  100.0  100.0  100.0  100.0  100.0  100.0  100.0  100.0  100.0  100.0  100.0  100.0)

[TRAJECTORY_GENERATION]
trajectory_type minimum_jerk

[COUPLING]
device                   couplingICubHandMk2
actuatedAxesNames        (l_hand_finger l_thumb_oppose l_thumb_proximal l_thumb_distal l_index_proximal l_index_distal l_middle_proximal l_middle_distal l_pinky)
actuatedAxesPosMin       (0.0  0.0  0.0  0.0   0.0  0.0   0.0  0.0   0.0)
actuatedAxesPosMax       (60.0 90.0 90.0 180.0 90.0 180.0 90.0 180.0 270.0)

[POSITION_CONTROL]
controlUnits  metric_units
controlLaw    joint_pid_gazebo_v1
kp            (0.1   0.1   0.1   0.1  0.1   0.1   0.1   0.1  0.1   0.1   0.1   0.1  0.1   0.1   0.1   0.1  0.1   0.1   0.1   0.1)
kd            (0.01  0.01  0.01  0.01 0.01  0.01  0.01  0.01 0.01  0.01  0.01  0.01 0.01  0.01  0.01  0.01 0.01  0.01  0.01  0.01)
ki            (0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0)
maxInt        (9999  9999  9999  9999 9999  9999  9999  9999 9999  9999  9999  9999 9999  9999  9999  9999 9999  9999  9999  9999)
maxOutput     (9999  9999  9999  9999 9999  9999  9999  9999 9999  9999  9999  9999 9999  9999  9999  9999 9999  9999  9999  9999)
shift         (0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0)
ko            (0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0)
stictionUp    (0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0)
stictionDwn   (0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0)

[VELOCITY_CONTROL]
velocityControlImplementationType integrator_and_position_pid
controlUnits  metric_units
controlLaw    joint_pid_gazebo_v1
kp            (8.726 8.726 8.726 5.235 8.726 8.726 8.726 5.235 8.726 8.726 8.726 8.726 8.726 8.726 8.726 8.726 8.726 8.726 8.726 8.726)
kd            (0.035 0.035 0.035 0.002 0.035 0.035 0.035 0.002 0.035 0.035 0.035 0.035 0.035 0.035 0.035 0.035 0.035 0.035 0.035 0.035)
ki            (0.002 0.002 0.002 0.0   0.002 0.002 0.002 0.0   0.002 0.002 0.002 0.002 0.002 0.002 0.002 0.002 0.002 0.002 0.002 0.002)
maxInt        (9999  9999  9999  9999  9999 9999  9999  9999  9999 9999  9999  9999  9999  9999  9999  9999  9999  9999  9999  9999)
maxOutput     (9999  9999  9999  9999  9999 9999  9999  9999  9999 9999  9999  9999  9999  9999  9999  9999  9999  9999  9999  9999)
shift         (0.0   0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0)
ko            (0.0   0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0)
stictionUp    (0.0   0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0)
stictionDwn   (0.0   0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0  0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0)

[IMPEDANCE_CONTROL]
controlUnits  metric_units
controlLaw    joint_pid_gazebo_v1
stiffness     (0.0   0.0   0.0   0.0 0.0   0.0   0.0   0.0 0.0   0.0   0.0   0.0 0.0   0.0   0.0   0.0 0.0   0.0   0.0   0.0)
damping       (0.0   0.0   0.0   0.0 0.0   0.0   0.0   0.0 0.0   0.0   0.0   0.0 0.0   0.0   0.0   0.0 0.0   0.0   0.0   0.0)

[LIMITS]
jntPosMax (0.0    0.0 20.0 20.0 90.0 90.0 90.0 90.0 90.0 90.0 90.0 90.0 90.0 90.0 90.0 90.0 90.0 90.0 90.0 90.0)
jntPosMin (-20.0  0.0  0.0 0.0  0.0  0.0  0.0  0.0  0.0  0.0  0.0  0.0  0.0  0.0  0.0  0.0  0.0  0.0  90.0 90.0)
jntVelMax (100.0  100.0 100.0 100.0 100.0 100.0 100.0 100.0 100.0 100.0 100.0 100.0 100.0 100.0 100.0 100.0 100.0 100.0 100.0 100.0)

```
