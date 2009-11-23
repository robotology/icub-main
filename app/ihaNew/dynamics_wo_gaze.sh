#!/bin/bash -l

source ./config_machines.sh
source ./config_ports.sh
source ./start_stop_utils.sh

# Eat the first (command) parameter
# This allows you to pass extra parameters to the command executed (e.g. ./script.sh start --dbg 40)
scriptcmd=$1
shift

################################################################################
# 
RUNNAME=dynamics
MACHINE_NAME=$DYNAMICS_MACHINE
MACHINE=${!MACHINE_NAME}
EXEC=ihaNewMotivationDynamics
NAME=$DYNAMICS_NAME
CONFIG_FILE=ihaMotivationDynamics.ini

# Set the paths for this machine (to be deprecated when ResourceFinder is finalized)
CNAME=$`echo $MACHINE_NAME"_CONFIG_PATH"`
CONFIG_PATH=`eval echo $CNAME`

XTERM_WRAP="xterm -hold -fg white -bg black -title dynamics -geometry $DYNAMICS_GEOM  -e bash -l -c "

# Command to start motivation dynamics without making connections to face/sound ports
CMD="$EXEC --name $DYNAMICS_NAME --file $CONFIG_PATH/$CONFIG_FILE --connect_to_data $SMI_INTERFACE_SENSORS_OUT --connect_to_expression $EXPRESSION_RAW_PORT $*"
#CMD="$EXEC --name $DYNAMICS_NAME --file $CONFIG_PATH/$CONFIG_FILE  $*"
# 
################################################################################


case "$scriptcmd" in
    kill)
		#kill_process $MACHINE ${EXEC##*/}
		stop_process $NAME $RUNNAME $MACHINE
		;;
    stop)
		#yarp disconnect $SOUNDSERVER_PORT_OUT $DYNAMICS_PORT_SOUND_IN
		#yarp disconnect $FACEDETECT_PORT_DETECT $DYNAMICS_PORT_FACE_IN
	        yarp disconnect $SMI_INTERFACE_SENSORS_OUT $DYNAMICS_PORT_DATA_IN
		stop_process $NAME $RUNNAME $MACHINE
		;;
	start_noxterm)
		start_process $RUNNAME $MACHINE "$CMD" "none" &
		;;
	start)
		start_process $RUNNAME $MACHINE "$CMD" "$XTERM_WRAP" &
		;;
	connect)
		yarp wait $DYNAMICS_PORT_DATA_IN
		yarp wait $SMI_INTERFACE_SENSORS_OUT
	        yarp connect $SMI_INTERFACE_SENSORS_OUT $DYNAMICS_PORT_DATA_IN
		;;
	disconnect)
	       yarp disconnect $SMI_INTERFACE_SENSORS_OUT $DYNAMICS_PORT_DATA_IN
		;;
    *)
		echo "usage: ${0##*/} [start|stop|kill|connect|disconnect]"
		;;
esac
