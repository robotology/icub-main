| Group name      | Parameter name     | Type            | Units   | Default Value  | Required | Description                                    | Notes                            |
|:---------------:|:------------------:|:---------------:|:-------:|:--------------:|:--------:|:----------------------------------------------:|:--------------------------------:|
|                 | jointNames         | vector<string>  | -       | -              |  Yes     | Names of the physical joints                   |                                  |
| LIMITS          | jntPosMin          | vector<double>  | -       | -              |  Yes     | Physical joints' position minimum             |                                  |
| LIMITS          | jntPosMax          | vector<double>  | -       | -              |  Yes     | Physical joints' position maximum             |                                  |
| COUPLING        | device             | string          | -       | -              |  Yes     | Name of the device that handles the coupling   |                                  |
| COUPLING        | actuatedAxesNames  | vector<string>  | -       | -              |  Yes     | Names of the actuated axes                     |                                  |
| COUPLING        | actuatedAxesPosMin | vector<double>  | -       | -              |  Yes     | Actuated axes' position minimum                |                                  |
| COUPLING        | actuatedAxesPosMax | vector<double>  | -       | -              |  Yes     | Actuated axes' position maximum                |                                  |
| COUPLING_PARAMS | L0x                | vector<double>  | -       | -              |  Yes     | x coordinate of the first end of the lever is  | The length of the list must be 5 |
| COUPLING_PARAMS | L0y                | vector<double>  | -       | -              |  Yes     | y coordinate of the first end of the lever is  | The length of the list must be 5 |
| COUPLING_PARAMS | q2bias             | vector<double>  | -       | -              |  Yes     | Angle of L1 - P1 when the finger is fully open | The length of the list must be 5 |
| COUPLING_PARAMS | q1off              | vector<double>  | -       | -              |  Yes     | Angle of P1 - P0 when the finger is closed     | The length of the list must be 5 |
| COUPLING_PARAMS | k                  | vector<double>  | -       | -              |  Yes     | Connecting rod length, L1-L0                   | The length of the list must be 5 |
| COUPLING_PARAMS | d                  | vector<double>  | -       | -              |  Yes     | Distance between the two joints, P1 and P0     | The length of the list must be 5 |
| COUPLING_PARAMS | l                  | vector<double>  | -       | -              |  Yes     | Distance between L1 and P1                     | The length of the list must be 5 |
| COUPLING_PARAMS | b                  | vector<double>  | -       | -              |  Yes     | Distance between L0 and P0                     | The length of the list must be 5 |