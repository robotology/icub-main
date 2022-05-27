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

And the wrapper:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<!DOCTYPE devices PUBLIC "-//YARP//DTD yarprobotinterface 3.0//EN" "http://www.yarp.it/DTD/yarprobotinterfaceV3.0.dtd">


    <device xmlns:xi="http://www.w3.org/2001/XInclude" name="battery_wrapper" type="multipleanalogsensorsserver">
        <param name="period">       10                  </param>
        <param name="name">       /icub/battery      </param>
        
        <action phase="startup" level="5" type="attach">
            <paramlist name="networks">
            <!-- The param value must match the device name in the corresponding body_part-ebX-jA_B-strain.xml file -->
                <elem name="FirstStrain">  battery </elem>
            </paramlist>
        </action>

        <action phase="shutdown" level="5" type="detach" />
    </device>

```

# 3. Yarp output

## 3.1. Ports

As for the wrapper file:

```bash
/<robot_name>/battery/measures:o 
/<robot_name>/battery/rpc:o

```

## 3.2. Output format

todo luca


# 4. Debug

## 4.1. Install folder
If you have already installed robotology-superbuild and you have cloned icub-main (and you are working here) overwrite robotology-superbuild binary like this in your icub-main:
```bash
CMAKE_INSTALL_PREFIX             <path_to_robotology>/robotology-superbuild/build/install
``` 

## 4.2. Visual studio code debugging
Use the following launch.json
```json
    {
        "name": "App debug battery sensor",
        "type": "cppdbg",
        "request": "launch",
        "program": "<absolute path to your robotology>/robotology-superbuild/build/install/bin/yarprobotinterface",
        "args": [],
        "stopAtEntry": false,
        "cwd": "<absolute path to your config files>/yarprobotinterface-config-multiple",
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