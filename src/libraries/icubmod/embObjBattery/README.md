# 1. General info
The device's purpose is to make it possible to connect up to the `CanBattery` sensor to a single EMS board.
This device works with the `bms` board. Note that the board can not be programmed for different message rates.

## 1.1. Prerequisite
The following modules should be included in the compilation process:

```cmake
ENABLE_icubmod_embObjCanBattery   ON
```

# 2. XML configuration file

This device should be used with XMLs like the following:

The root XML:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<!DOCTYPE robot PUBLIC "-//YARP//DTD yarprobotinterface 3.0//EN" "http://www.yarp.it/DTD/yarprobotinterfaceV3.0.dtd">

    <robot name="testFT" build="1" portprefix="icub" xmlns:xi="http://www.w3.org/2001/XInclude">
        <params>
    <xi:include href="hardware/electronics/pc104.xml" />
        </params>
        
    <devices>
 
 
    <!-- ANALOG SENSOR CANBATTERY -->
    <xi:include href="wrappers/CanBattery/testCanBattery_wrapper.xml" />    
    <xi:include href="hardware/CanBattery/eb1-j0-strain2-canbattery.xml" /> 
  
      
    </devices>
</robot> 
```

The device XML in `hardware\canBattery` (refer to the root XML file):
```xml
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE device PUBLIC "-//YARP//DTD yarprobotinterface 3.0//EN" "http://www.yarp.it/DTD/yarprobotinterfaceV3.0.dtd">

<device xmlns:xi="http://www.w3.org/2001/XInclude" name="canbattery" type="embObjBattery">

    <xi:include href="../../general.xml" />

    <!-- This value must match the net board name in the corresponding hardware/electronics/... file -->
    <xi:include href="../electronics/left_arm-eb1-j0_1-eln.xml" />

    <group name="SERVICE">

        <param name="type"> eomn_serv_AS_battery </param>

        <group name="PROPERTIES">

            <group name="CANBOARDS">
                <param name="type">                 bms      </param>

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
                <param name="board">                bms             </param>
                <param name="location">             CAN2:13         </param>
            </group>

        </group>


        <group name="SETTINGS">
            <param name="enabledSensors">            battery1        </param>
            <param name="acquisitionRate">           1000            </param>   <!-- msec -->
        </group>


    </group>

</device>
```

And the XML wrapper in `.\wrappers\canBattery` (refer to the root XML file)::

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<!DOCTYPE devices PUBLIC "-//YARP//DTD yarprobotinterface 3.0//EN" "http://www.yarp.it/DTD/yarprobotinterfaceV3.0.dtd">
<device xmlns:xi="http://www.w3.org/2001/XInclude" name="battery_wrapper" type="batteryWrapper">
    <param name="period">       1                  </param> 
    <param name="name">       /icub/battery      </param>

    <action phase="startup" level="5" type="attach">
        <paramlist name="networks">
        <!-- This value must match the device name in the corresponding hardware/battery/canBattery.xml file -->
            <elem name="battery">  canbattery </elem>
        </paramlist>
    </action>

    <action phase="shutdown" level="5" type="detach" />
</device>
```

# 3. Yarp output

## 3.1. Ports

As for the wrapper file:

```bash
/<robot_name>/battery/data:o 
/<robot_name>/battery/rpc:o

```

## 3.2. Output format

The output from bash commend
```bash
 yarp read ... /icub/battery/data:o
```

`40 3.300000011920928955078 80.0 31.0 0`  
`voltage -- current -- charge -- temperature -- status`

Voltage in Volt
Current in Amper
Charge in percent
Temperature in degree Celtius
Status not used for now

# 4. Battery client
To receive data from the device you should use the iBattery interface.
See a complete example here:
https://github.com/robotology/yarp/tree/master/src/yarpbatterygui

# 5. Debug

## 5.1. Install folder
If you have already installed robotology-superbuild and you have cloned icub-main (and you are working here) overwrite robotology-superbuild binary like this in your icub-main:
```bash
CMAKE_INSTALL_PREFIX             <path_to_robotology>/robotology-superbuild/build/install
``` 

## 5.2. Visual studio code debugging
Use the following launch.json
```JSON
    {
    "version": "0.2.0",
    "configurations": [
    {
            "name": "App debug battery sensor",
            "type": "cppdbg",
            "request": "launch",
            "program": "<robotology-path>/robotology-superbuild/build/install/bin/yarprobotinterface",
            "args": [],
            "stopAtEntry": false,
            "cwd": "<xmlconfig-files>/yarprobotinterface-config-canbattery",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                },
                {
                    "description": "Set Disassembly Flavor to Intel",
                    "text": "-gdb-set disassembly-flavor intel",
                    "ignoreFailures": true
                }
            ],
            "miDebuggerPath": "/usr/bin/gdb",
        },
        {
        "name": "UT debug battery sensor",
        "type": "cppdbg",
        "request": "launch",
        "program": "/home/triccyx/Documents/robotology-superbuild/build/install/bin/unittest",
        "args": [],
        "stopAtEntry": false,
        "cwd": "/home/triccyx/Documents/robotology-superbuild/build/install/bin",
        "environment": [],
        "externalConsole": false,
        "MIMode": "gdb",
        "setupCommands": [
            {
                "description": "Enable pretty-printing for gdb",
                "text": "-enable-pretty-printing",
                "ignoreFailures": true
            },
            {
                "description": "Set Disassembly Flavor to Intel",
                "text": "-gdb-set disassembly-flavor intel",
                "ignoreFailures": true
            }
        ],
        "miDebuggerPath": "/usr/bin/gdb",
    }
    ]
}
```
Also select as build type DEBUG

# 6. Unittest 
The device has a set of unittest that can be activated with:
```
BUILD_TESTING                    ON
```