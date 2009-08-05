#!/bin/bash

MYSHELL="bash -c"

case "$1" in
	kill)
		killall -9 DrumGeneratorModule
		sleep 0.2
		killall -9 clockModule
		sleep 0.2
		;;
    stop)
		killall DrumGeneratorModule
		sleep 0.2
		killall clockModule
		sleep 0.2
		;;
	start)
		$MYSHELL "$ICUB_ROOT/src/drummingEPFL/GeneralClock/clockModule --period 20 --part head &"
		sleep 1
		$MYSHELL "$ICUB_ROOT/src/drummingEPFL/DrumGenerator/DrumGeneratorModule --part head --file $ICUB_ROOT/src/drummingEPFL/config/headConfig.ini &"
		sleep 1
		$MYSHELL "sudo $ICUB_ROOT/src/drummingEPFL/DrumGenerator/DrumGeneratorModuleRightArm"
		sleep 1
		$MYSHELL "$ICUB_ROOT/src/drummingEPFL/DrumGenerator/DrumGeneratorModule --part left_arm --file $ICUB_ROOT/src/drummingEPFL/config/left_armConfig.ini &"
		sleep 1
		$MYSHELL "$ICUB_ROOT/src/drummingEPFL/DrumGenerator/DrumGeneratorModule --part right_leg --file $ICUB_ROOT/src/drummingEPFL/config/right_legConfig.ini &"
		sleep 1
		$MYSHELL "$ICUB_ROOT/src/drummingEPFL/DrumGenerator/DrumGeneratorModule --part left_leg --file $ICUB_ROOT/src/drummingEPFL/config/left_legConfig.ini &"
		;;

    *)
		echo "stop, start, kill"
		;;
esac

# --part right_arm --file 
#$ICUB_ROOT/src/drummingEPFL/config/right_armConfig.ini &"
