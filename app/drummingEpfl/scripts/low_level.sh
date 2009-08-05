

#!/bin/bash

MYSHELL="bash -c"

case "$1" in
	kill)
		killall -9 DrumGeneratorModule
		sleep 0.2
		killall -9 clockModule
		sleep 0.2
		killall -9 velocityControl
		sleep 0.2
		;;
    stop)
		killall DrumGeneratorModule
		sleep 0.2
		killall clockModule
		sleep 0.2
		killall velocityControl
		sleep 0.2
		;;
	start)
		$MYSHELL "$ICUB_ROOT/src/velocityControl/velocityControl --period 20 --part right_arm &"
		$MYSHELL "$ICUB_ROOT/src/velocityControl/velocityControl --period 20 --part left_arm &"
		$MYSHELL "$ICUB_ROOT/src/velocityControl/velocityControl --period 20 --part head &"
		$MYSHELL "$ICUB_ROOT/src/velocityControl/velocityControl --period 20 --part right_leg &"
		$MYSHELL "$ICUB_ROOT/src/velocityControl/velocityControl --period 20 --part left_leg &"
		sleep 2.5
		$MYSHELL "$ICUB_ROOT/src/drummingEPFL/GeneralClock/clockModule --period 20 --part head &"
		sleep 1
		$MYSHELL "$ICUB_ROOT/src/drummingEPFL/DrumGenerator/DrumGeneratorModule --part head --file $ICUB_ROOT/src/drummingEPFL/config/headConfig.ini &"
		$MYSHELL "$ICUB_ROOT/src/drummingEPFL/DrumGenerator/DrumGeneratorModule --part right_arm --file $ICUB_ROOT/src/drummingEPFL/config/right_armConfig.ini &"
		$MYSHELL "$ICUB_ROOT/src/drummingEPFL/DrumGenerator/DrumGeneratorModule --part left_arm --file $ICUB_ROOT/src/drummingEPFL/config/left_armConfig.ini &"
		$MYSHELL "$ICUB_ROOT/src/drummingEPFL/DrumGenerator/DrumGeneratorModule --part right_leg --file $ICUB_ROOT/src/drummingEPFL/config/right_legConfig.ini &"
		$MYSHELL "$ICUB_ROOT/src/drummingEPFL/DrumGenerator/DrumGeneratorModule --part left_leg --file $ICUB_ROOT/src/drummingEPFL/config/left_legConfig.ini &"
		sleep 1
		;;

    *)
		echo "stop, start, kill"
		;;
esac
