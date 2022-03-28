# 1. General info
The device's purpose is to make it possible to connect up to 4 FT sensors to a single EMS board.

# 2. XML configuration file

This device should be used with an XML like the following:

```xml
    <group name="SERVICE">
        
        <param name="type"> eomn_serv_AS_ft </param>

        <group name="PROPERTIES">

            <!-- we can have as many boards as we want, 
                but now the only good ones are strain and strain2. 
                use eoas_ft_isboardvalid() to verify -->
            <group name="CANBOARDS">
                <param name="type">                 strain2     strain      </param>

                <group name="PROTOCOL">
                    <param name="major">            2           1           </param>    
                    <param name="minor">            0           0           </param>     
                </group>                    
                <group name="FIRMWARE">
                    <param name="major">            2           1           </param>    
                    <param name="minor">            0           1           </param> 
                    <param name="build">            9           3           </param>
                </group>
            </group>
            
            <!-- we can have as many sensors as we want. however ... -->
            <group name="SENSORS">
                <param name="id">                   l_foot_ft1      l_foot_ft2      l_foot_ft3  </param>
                <param name="board">                strain2         strain2         strain      </param>
                <param name="location">             CAN2:13         CAN1:12         CAN2:11     </param>
            </group>                
        
        </group>

        <!-- we can pick up at max 4 sensors from the above list. 
            we can specify parameters that are different for each sensor 
            - ftPeriod is expressed in ms in the range [1, 250]
            - temperaturePeriod is expressed in ms (but it will be used in seconds).
            - useCalibration is boolean
            -->
        <group name="SETTINGS">        
            <param name="enabledSensors">       l_foot_ft1  l_foot_ft2  l_foot_ft3      </param>
            <param name="ftPeriod">             10          10          10              </param>
            <param name="temperaturePeriod">    1000        1000        0               </param>
            <param name="useCalibration">       true        false       true            </param>           
        </group>       
    
        <group name="CANMONITOR">        
            <param name="checkPeriod">      100     </param>
            <param name="reportMode">       ALL     </param>
            <param name="ratePeriod">       20*1000 </param>
        </group>    
    </group>

```

## 2.1. Raw ADC data or FT data
To log raw data directly from ADC output use the param with the name `useCalibration`

```xml
<param name="useCalibration">       true        false       true            </param>           
```

if `false` raw data are logged from the correspondent sensor.


# 3. Yarp output

## 3.1. Ports

```bash
/<robot_name>/ADCs/measures:o 
/<robot_name>/ADCs/rpc:o
/<robot_name>/imu/measures:o
/<robot_name>/imu/rpc:o

```

## 3.2. Format

# 1. General info
The device's purpose is to make it possible to connect up to 4 FT sensors to a single EMS board.

# 2. XML configuration file

This device should be used with an XML like the following:

```xml
    <group name="SERVICE">
        
        <param name="type"> eomn_serv_AS_ft </param>

        <group name="PROPERTIES">

            <!-- we can have as many boards as we want, 
                but now the only good ones are strain and strain2. 
                use eoas_ft_isboardvalid() to verify -->
            <group name="CANBOARDS">
                <param name="type">                 strain2     strain      </param>

                <group name="PROTOCOL">
                    <param name="major">            2           1           </param>    
                    <param name="minor">            0           0           </param>     
                </group>                    
                <group name="FIRMWARE">
                    <param name="major">            2           1           </param>    
                    <param name="minor">            0           1           </param> 
                    <param name="build">            9           3           </param>
                </group>
            </group>
            
            <!-- we can have as many sensors as we want. however ... -->
            <group name="SENSORS">
                <param name="id">                   l_foot_ft1      l_foot_ft2      l_foot_ft3  </param>
                <param name="board">                strain2         strain2         strain      </param>
                <param name="location">             CAN2:13         CAN1:12         CAN2:11     </param>
            </group>                
        
        </group>

        <!-- we can pick up at max 4 sensors from the above list. 
            we can specify parameters that are different for each sensor 
            - ftPeriod is expressed in ms in the range [1, 250]
            - temperaturePeriod is expressed in ms (but it will be used in seconds).
            - useCalibration is boolean
            -->
        <group name="SETTINGS">        
            <param name="enabledSensors">       l_foot_ft1  l_foot_ft2  l_foot_ft3      </param>
            <param name="ftPeriod">             10          10          10              </param>
            <param name="temperaturePeriod">    1000        1000        0               </param>
            <param name="useCalibration">       true        false       true            </param>           
        </group>       
    
        <group name="CANMONITOR">        
            <param name="checkPeriod">      100     </param>
            <param name="reportMode">       ALL     </param>
            <param name="ratePeriod">       20*1000 </param>
        </group>    
    </group>

```

## 2.1. Raw ADC data or FT data
To log raw data directly from ADC output use the param with the name `useCalibration`

```xml
<param name="useCalibration">       true        false       true            </param>           
```
if `false` raw data are logged from the correspondent sensor.


# 3. Yarp output port

Output example:  
```
1277 1648477185.597940 () () () () (((32.0) 3062439114.0)) (((30.0 30.0 60.0 -15.0 -14.0 -13.0) 3062439114.0))()()()()()  
```

3062439114.0 **is the board timestamp**  
(32.0) **is the temperature**    
(30.0 30.0 60.0 -15.0 -14.0 -13.0) **are the FT data**  
