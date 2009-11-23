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
RUNNAME=mobileeye
MACHINE_NAME=$MOBEYE_MACHINE
MACHINE=${!MACHINE_NAME}
EXEC=ihaNewMobileEye
NAME=$MOBEYE_NAME
CONFIG_FILE=ihaMobileEye.ini

# Set the paths for this machine (to be deprecated when ResourceFinder is finalized)
CNAME=$`echo $MACHINE_NAME"_CONFIG_PATH"`
CONFIG_PATH=`eval echo $CNAME`
XTERM_WRAP="xterm -hold -fg white -bg black -title mobileeye -geometry $DYNAMICS_GEOM  -e bash -l -c "
#XTERM_WRAP="xterm -fg white -bg black -title salience "
CMD="$EXEC --name $MOBEYE_NAME --file $CONFIG_PATH/$CONFIG_FILE"
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
		start_process $RUNNAME $MACHINE "$CMD" "$XTERM_WRAP" &
		;;
	connect)
	        #yarp wait $MOBEYESERVER_PORT
		yarp wait $MOBEYE_PORT_GAZE_IN
		yarp wait $DVGRAB_NAME
		yarp wait $MOBEYE_PORT_IN
		yarp wait $SELFFACEDETECT_PORT_DETECT
		# connect the camera RGB output to the face detector
		yarp connect $DVGRAB_NAME $MOBEYE_PORT_IN 
		yarp connect $MOBEYESERVER_PORT $MOBEYE_PORT_GAZE_IN
		yarp connect $SELFFACEDETECT_PORT_DETECT $MOBEYE_PORT_FACE_IN
		;;
    *)
		echo "usage: ${0##*/} [start|stop|connect|kill]"
		;;
esac
