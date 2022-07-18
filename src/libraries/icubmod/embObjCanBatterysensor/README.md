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

The device XML in `hardware\CanBattery` (refer to the root XML file):
```xml
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE device PUBLIC "-//YARP//DTD yarprobotinterface 3.0//EN" "http://www.yarp.it/DTD/yarprobotinterfaceV3.0.dtd">

<device xmlns:xi="http://www.w3.org/2001/XInclude" name="eb1-j0-strain2-canbattery" type="embObjCanBatterysensor">

    <xi:include href="../../general.xml" />

    <xi:include href="../../hardware/electronics/eb1-j0-eln.xml" />

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

And the XML wrapper in `.\wrappers\CanBattery` (refer to the root XML file)::

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<!DOCTYPE devices PUBLIC "-//YARP//DTD yarprobotinterface 3.0//EN" "http://www.yarp.it/DTD/yarprobotinterfaceV3.0.dtd">


    <device xmlns:xi="http://www.w3.org/2001/XInclude" name="battery_wrapper" type="batteryWrapper">
        <param name="period">       1                  </param> 
        <param name="name">       /icub/battery      </param>
        
        <action phase="startup" level="5" type="attach">
            <paramlist name="networks">
            <!-- The param value must match the device name in the corresponding body_part-ebX-jA_B-strain.xml file -->
                <elem name="battery">  eb1-j0-strain2-canbattery </elem>
            </paramlist>
        </action>

        <action phase="shutdown" level="5" type="detach" />
    </device>
```

Important note:  
`<elem name="battery">  eb1-j0-strain2-canbattery </elem>`
should contains the device name `eb1-j0-strain2-canbattery` of the device file.
 

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

`0.400000005960464477539 0.300000011920928955078 80.0 31.0 0`  
`voltage -- current -- charge -- temperature -- status`


# 4. Debug

## 4.1. Install folder
If you have already installed robotology-superbuild and you have cloned icub-main (and you are working here) overwrite robotology-superbuild binary like this in your icub-main:
```bash
CMAKE_INSTALL_PREFIX             <path_to_robotology>/robotology-superbuild/build/install
``` 

## 4.2. Visual studio code debugging
Use the following launch.json
```JSON
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
        }
```

# 5. Unittest 
The device has a set of unittest.