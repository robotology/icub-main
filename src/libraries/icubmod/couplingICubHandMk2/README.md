![YARP logo](https://raw.githubusercontent.com/robotology/yarp/master/doc/images/yarp-robot-24.png "yarp-device-realsense2")
## couplingICubHandMk2


This is the device for [YARP](https://www.yarp.it/) for handling the coupling of the [hands mk5](https://icub-tech-iit.github.io/documentation/hands/hands_mk5_coupling/)

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
|  `COUPLING_PARAMS`                      |     -             | group         | Read / write | -       |   -           |  Yes            | The group containing the coupling params                                   |                                   |
|                      |     `L0x`        | list of double     | Read / write | -       |   -           |  Yes            | x coordinate of the first end of the lever is applied                                      |           The lenght of the list must be 5                        |
|                      |     `L0y`        | list of double     | Read / write | -       |   -           |  Yes            | y coordinate of the first end of the lever is applied                                      |           The lenght of the list must be 5                        |
|                      |     `q2bias`        | list of double     | Read / write | -       |   -           |  Yes            | Angle of L1 - P1  when the finger is fully open           |           The lenght of the list must be 5                        |
|                      |     `q1off`        | list of double     | Read / write | -       |   -           |  Yes            | Angle of P1 - P0  when the finger is fully closed           |           The lenght of the list must be 5                        |
|                      |     `k`        | list of double     | Read / write | -       |   -           |  Yes            | Connecting rod length, \|L1-L0\|           |           The lenght of the list must be 5                        |
|                      |     `d`        | list of double     | Read / write | -       |   -           |  Yes            | Distance between the two joints, P1 and P0           |           The lenght of the list must be 5                        |
|                      |     `l`        | list of double     | Read / write | -       |   -           |  Yes            | Distance between L1 and P1            |           The lenght of the list must be 5                        |
|                      |     `b`        | list of double     | Read / write | -       |   -           |  Yes            | Distance between L0 and P0             |           The lenght of the list must be 5                        |



Configuration file using `.ini` format:

```ini
device                couplingICubHandMk2
jointNames l_thumb_add l_thumb_prox l_thumb_dist l_index_add l_index_prox l_index_dist l_middle_prox l_middle_dist l_ring_prox l_ring_dist l_pinkie_prox l_pinkie_dist

[COUPLING]
actuatedAxesNames     l_thumb_add l_thumb_oc l_index_add l_index_oc l_middle_oc l_ring_pinky_oc
actuatedAxesPosMin       0.0 0.0 0.0 0.0 0.0 0.0
actuatedAxesPosMax       90.0 82.1 15.0 90.0 90.0 90.0

[COUPLING_PARAMS]
L0x     -0.00555 -0.0050 -0.0050 -0.0050  -0.0050
L0y      0.00285  0.0040  0.0040  0.0040   0.0040
q2bias     -180.0  -173.35  -173.35  -173.35  -170.54
q1off       4.29    2.86    2.86    2.86     3.43
k         0.0171 0.02918 0.02918 0.02918  0.02425
d        0.02006 0.03004 0.03004 0.03004  0.02504
l         0.0085 0.00604 0.00604 0.00604  0.00608
b        0.00624  0.0064  0.0064  0.0064   0.0064


[LIMITS]
jntPosMax   90.0  82.1  53.6  15.0  90.0  99.2  90.0  99.2  90.0  99.2  90.0  93.3
jntPosMin    0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0   0.0
```
