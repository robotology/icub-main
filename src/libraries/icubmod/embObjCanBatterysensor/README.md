# 1. General info
The device's purpose is to make it possible to connect up to `1 CanBattery` sensor to a single EMS board.

## 1.1. Prereqisite
The following modules should be included in the compilation process:

```cmake
ENABLE_icubmod_canBusAnalogSen   ON
ENABLE_icubmod_embObjCanBattery   ON
```

# 2. XML configuration file

This device should be used with an XML like the following:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE device PUBLIC "-//YARP//DTD yarprobotinterface 3.0//EN" "http://www.yarp.it/DTD/yarprobotinterfaceV3.0.dtd">


<device xmlns:xi="http://www.w3.org/2001/XInclude" name="battery" type="embObjCanBatterysensor">

    <xi:include href="../../general.xml" />

    <xi:include href="../../hardware/electronics/face-eb22-j0_1-eln.xml" />

    <group name="SERVICE">

        <param name="type"> eomn_serv_AS_canbattery </param>

        <group name="PROPERTIES">

            <!-- we can have as many boards as we want, 
                 but now the only good ones are strain and strain2. 
                 use eoas_ft_isboardvalid() to verify -->
            <group name="CANBOARDS">
                <param name="type">                 canbattery      </param>

                <group name="PROTOCOL">
                    <param name="major">            0                   </param>
                    <param name="minor">            0                      </param>
                </group>
                <group name="FIRMWARE">
                    <param name="major">            0                   </param>
                    <param name="minor">            0                   </param>
                    <param name="build">            0                   </param>
                </group>
            </group>

            <group name="SENSORS">
                <param name="id">                   battery1        </param>
                <param name="board">                canbattery             </param>
                <param name="location">             CAN2:13         </param>
            </group>

        </group>

        <!-- we can pick up at max 4 sensors from the above list. 
             we can specify parameters which are different for each sensor 
             ft rate is in ms in range [1, 250]
             temperature rate is in ms but must be transformed in seconds.
             use calibration is boolean
             -->
        <group name="SETTINGS">
            <param name="enabledSensors">            battery1        </param>
            <param name="acquisitionRate">           1000            </param>   <!-- msec -->
        </group>


    </group>

</device>
```

luca todo


# 3. Yarp output

## 3.1. Ports

todo luca

## 3.2. Output format

todo luca