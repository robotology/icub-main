# 1. General info
The device's purpose is to make it possible to connect up to `4 FT` sensors to a single EMS board.

## 1.1. Prereqisite
The following modules should be included in the compilation process:

```cmake
ENABLE_icubmod_canBusAnalogSen   ON
ENABLE_icubmod_embObjMultipleF   ON
```

# 2. XML configuration file

This device should be used with an XML like the following:

```xml
    <?xml version="1.0" encoding="UTF-8"?>
    <!DOCTYPE devices PUBLIC "-//YARP//DTD yarprobotinterface 3.0//EN" "http://www.yarp.it/DTD/yarprobotinterfaceV3.0.dtd">


    <device xmlns:xi="http://www.w3.org/2001/XInclude" name="eb1-j0-strain2-multi" type="embObjMultipleFTsensors">

        <xi:include href="../../general.xml" />

        <xi:include href="../../hardware/electronics/eb1-j0-eln.xml" />

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
    </device>
```

```xml
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE devices PUBLIC "-//YARP//DTD yarprobotinterface 3.0//EN" "http://www.yarp.it/DTD/yarprobotinterfaceV3.0.dtd">


<device xmlns:xi="http://www.w3.org/2001/XInclude" name="testFT_wrapper" type="multipleanalogsensorsserver">
    <param name="period">      10                           </param>
    <param name="name">       /testFT/ADCs      </param>

    <action phase="startup" level="5" type="attach">
        <paramlist name="networks">
            <!-- The param value must match the device name in the corresponding body_part-ebX-jA_B-strain.xml file -->
            <elem name="FirstStrain"> eb1-j0-strain2-multi </elem>
        </paramlist>
    </action>

    <action phase="shutdown" level="5" type="detach" />
</device>
```

## 2.1. Raw ADC data or FT data
To log raw data directly from ADC output use the param with the name `useCalibration`

```xml
<param name="useCalibration">       true        false       true            </param>           
```

if `false` raw data are logged from the correspondent sensor.


# 3. Yarp output

## 3.1. Ports

All sensors are logged on the same port.  

```bash
/<robot_name>/ADCs/measures:o 
/<robot_name>/ADCs/rpc:o

```

## 3.2. Output format

Output example for one sensor:  
```
1277 1648477185.597940 () () () () (((32.0) 3062439114.0)) (((30.0 30.0 60.0 -15.0 -14.0 -13.0) 3062439114.0))()()()()()  
```

3062439114.0 **is the board timestamp**  
(32.0) **is the temperature**    
(30.0 30.0 60.0 -15.0 -14.0 -13.0) **are the FT data**  

For two sensors:
```
4714 1648546986.847330 () () () () (((31.0) 8627844988.0) ((33.0) 8627845987.0)) (((30.0 0.0 -60.0 -1.0 -1.0 -1.0) 8627844988.0) ((0.256134033203125 0.66009521484375 -3.583465576171875 0.0064697265625 0.067291259765625 -0.043487548828125) 8627845987.0)) () () () ()
```
8627844988.0 **is the board timestamp for first sensor**  
(31.0) **is the temperature first sensor**    
(30.0 0.0 -60.0 -1.0 -1.0 -1.0) **are the FT data first sensor**  

8627845987.0 **is the board timestamp for second sensor**  
(33.0) **is the temperature second sensor**    
(0.256134033203125 0.66009521484375 -3.583465576171875 0.0064697265625 0.067291259765625 -0.043487548828125) 8627845987.0) **are the FT data second sensor**  