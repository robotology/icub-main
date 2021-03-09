# bcbBattery



This module implements the [``IBattery``](http://www.yarp.it/git-master/classyarp_1_1dev_1_1IBattery.html) interface to retrieve the status of the battery in the iCub backpack.



## Dependencies

In order for this module to be compiled, it needs to be enabled with the option ``ENABLE_icubmod_bcbBattery`` set to ``ON``. In addition, ``yarp`` needs to be compiled with the following options set to ``ON``:

- ``ENABLE_yarpmod_batteryWrapper``
- ``ENABLE_yarpmod_serial``
- ``ENABLE_yarpmod_serialport``

In addition, in order to have a GUI representation of the battery status, enable the following in ``yarp``

- ``ENABLE_yarpmod_batteryClient``
- ``YARP_COMPILE_yarpbatterygui``

## How to use

This module can be used by resorting to the configuration files already available in the [``battery`` folder](https://github.com/robotology/robots-configuration/tree/master/experimentalSetups/battery) in the ``experimentalSetups`` of ``robots-configuration``. From that folder, it is sufficient to run ``yarprobotinterface``.



## Configuration parameters

The configuration file of this module needs two groups. A group named ``SERIAL_PORT`` with all the parameters relative to the serial port connection. For the meaning of these parameters refer to the ``serialPort`` [header file](https://github.com/robotology/yarp/blob/ab488c74e37de629af207b83be2fd90ae7412b1b/src/devices/serialport/SerialDeviceDriver.h#L34-L70). Their setup depends on the serial configuration done on the hardware side.



The second group should be named ``GENERAL`` and should contain the following:

- ``thread_period``:    the thread period in milliseconds 
- ``screen``: when equal to 1, it enables an info print in the terminal showing the battery status
- ``verbose``: when equal to 1, it prints on the terminal the raw value coming from the terminal.
- ``silence_sync_warnings``: when 1 it avoids printing warning messages relative to syncing.

