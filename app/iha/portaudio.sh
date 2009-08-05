#!/bin/bash

source ./config_machines.sh
source ./config_ports.sh
source ./start_stop_utils.sh

# Eat the first (command) parameter
# This allows you to pass extra parameters to the command executed (e.g. ./script.sh start --dbg 40)
scriptcmd=$1
shift

################################################################################
# 
RUNNAME=pa
MACHINE_NAME=$PORTAUDIO_MACHINE
MACHINE=${!MACHINE_NAME}
NAME=$PORTAUDIO_NAME
EXEC=yarpdev

# Set the paths for this machine (to be deprecated when ResourceFinder is finalized)
CNAME=$`echo $MACHINE_NAME"_CONFIG_PATH"`
CONFIG_PATH=`eval echo $CNAME`

CMD="yarpdev --device sound_grabber --subdevice portaudio --rate 11205 --samples 256 --framerate 20 --name $PORTAUDIO_PORT --read $*"
# 
################################################################################

case "$scriptcmd" in
    kill)
		kill_process $MACHINE ${EXEC##*/}
		#stop_process $NAME $RUNNAME $MACHINE
		;;
    stop)
		stop_process $NAME $RUNNAME $MACHINE
		;;
	start)
		start_process $RUNNAME $MACHINE "$CMD" "none" &
		;;
    *)
		echo "usage: ${0##*/} [start|stop|kill]"
		;;
esac
