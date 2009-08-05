#!/bin/bash

MYSHELL="xterm -e "

case "$1" in
	kill)
		killall -9 demoAff
		sleep 0.2
		killall -9 artoolkittracker
		sleep 0.2
		killall -9 camshiftplus
		sleep 0.2
		killall -9 camCalib
		sleep 0.2
		;;
    stop)
		killall -9 demoAff
		sleep 0.2
		killall -9 artoolkittracker
		sleep 0.2
		killall -9 camshiftplus
		sleep 0.2
		killall -9 camCalib
		sleep 0.2
		;;
	start)
		$MYSHELL "$ICUB_ROOT/bin/demoAff --file $ICUB_ROOT/conf/BNaffordances.txt" &
		$MYSHELL "$ICUB_ROOT/bin/artoolkittracker --file  $ICUB_ROOT/conf/iCubARMarkerDetectorModule.ini" &
		$MYSHELL "$ICUB_ROOT/bin/camshiftplus" &
		$MYSHELL "$ICUB_ROOT/bin/camCalib --file  $ICUB_ROOT/conf/icubCamLeftIntrinsics.ini" &
		;;

    *)
		echo "stop, start, kill"
		;;
esac
