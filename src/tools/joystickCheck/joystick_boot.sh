#!/bin/sh
joystickCheck
if [ "$?" -eq "1" ]; then
    echo "joystick boot requested"
    yarp server
    sleep 2
    iCubInterface --context iKart --config conf/iKart.ini &
    sleep 2
    iKartCtrl --no_laser --no_compass &
    sleep 2
    joystickCtrl --context joystickCtrl --from conf/xbox_linux.ini &
    sleep 2
    yarp connect /joystickCtrl:o /ikart/joystick:i
else
    echo "No joystick boot requested"
fi

