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
RUNNAME=facedetect
MACHINE_NAME=$FACEDETECT_MACHINE
MACHINE=${!MACHINE_NAME}
EXEC=ihaIhaFaceDetect
NAME=$FACEDETECT_NAME
CONFIG_FILE=ihaIhaFaceDetect.ini
CASCADE1=haarcascade_frontalface_alt2.xml

# Set the paths for this machine (to be deprecated when ResourceFinder is finalized)
CNAME=$`echo $MACHINE_NAME"_CONFIG_PATH"`
CONFIG_PATH=`eval echo $CNAME`

CMD="$EXEC --name $FACEDETECT_NAME --CASCADES \"(cascade1 $CONFIG_PATH/$CASCADE1)\" $*"
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
	connect)
		yarp wait $FACEDETECT_PORT_IN
		# connect the camera RGB output to the face detector
		yarp connect $CAMERA $FACEDETECT_PORT_IN
		;;
    *)
		echo "usage: ${0##*/} [start|stop|connect|kill]"
		;;
esac
