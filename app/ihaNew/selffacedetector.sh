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
RUNNAME=selffacedetect
MACHINE_NAME=$SELFFACEDETECT_MACHINE
MACHINE=${!MACHINE_NAME}
EXEC=ihaNewIhaFaceDetect
NAME=$SELFFACEDETECT_NAME
CONFIG_FILE=ihaIhaSelfFaceDetect.ini
CASCADE1=haarcascade_frontalface_alt2.xml

# Set the paths for this machine (to be deprecated when ResourceFinder is finalized)
CNAME=$`echo $MACHINE_NAME"_CONFIG_PATH"`
CONFIG_PATH=`eval echo $CNAME`
#XTERM_WRAP="xterm -fg white -bg black -title facedetect "
XTERM_WRAP="xterm -hold -fg white -bg black -title selffacedetect -geometry $DYNAMICS_GEOM  -e bash -l -c "
CMD="$EXEC --name $SELFFACEDETECT_NAME --output_image FALSE --CASCADES \"(cascade1 $CONFIG_PATH/$CASCADE1)\" $*"
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
	start_noxterm)
		start_process $RUNNAME $MACHINE "$CMD" "none" &
		;;
	start)
		start_process $RUNNAME $MACHINE "$CMD" "$XTERM_WRAP" &
		;;
	connect)
		yarp wait $SELFFACEDETECT_PORT_IN
		# connect the camera RGB output to the face detector
		yarp connect $DVGRAB_NAME $SELFFACEDETECT_PORT_IN 
		#yarp connect $OPENCV_NAME $SELFFACEDETECT_PORT_IN
		;;
    *)
		echo "usage: ${0##*/} [start|stop|connect|kill]"
		;;
esac
