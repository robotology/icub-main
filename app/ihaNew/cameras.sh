#!/bin/bash

source ./config_ports.sh

case "$1" in
    stop)
		yarp terminate $DRAGONFLY_LEFT_NAME/quit
		;;
	start)
		yarpdev --device grabber --subdevice dragonfly --d 0 --width 320 --height 240 --framerate 33 --white_balance 0.55 0.61 --gain 0.6 --shutter 0.7 --name ${DRAGONFLY_LEFT_NAME} --DR2 --verbose &
		;;
    *)
		echo "usage: ${0##*/} [start|stop]"
		;;
esac
