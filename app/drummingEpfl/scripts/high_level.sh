#!/bin/bash

MYSHELL="gnome-terminal -e"

case "$1" in
	kill)
		killall -9 guiDemo
		sleep 0.2
		killall -9 DrumManager
		sleep 0.2
		;;
    stop)
		killall guiDemo
		sleep 0.2
		killall DrumManager
		sleep 0.2
		;;
	start)
		$MYSHELL "$ICUB_ROOT/src/drummingEPFL/DrumManager/DrumManager --config-path $ICUB_ROOT/src/drummingEPFL/config &"
		sleep 2
		$MYSHELL "$ICUB_ROOT/src/drummingEPFL/guiDemo3/guiDemo 
&"
		sleep 1
		;;

    *)
		echo "stop, start, kill"
		;;
esac
