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
RUNNAME=mobileyeserver
MACHINE_NAME=$MOBEYESERVER_MACHINE
MACHINE=${!MACHINE_NAME}
EXEC=ihaNewMobileEyeSensor
NAME=$MOBEYESERVER_NAME
CONFIG_FILE=ihaMobileEyeSensor.ini

# Set the paths for this machine (to be deprecated when ResourceFinder is finalized)
CNAME=$`echo $MACHINE_NAME"_CONFIG_PATH"`
CONFIG_PATH=`eval echo $CNAME`

XTERM_WRAP="xterm -fg white -bg black -title mobileeyeserver -geometry $MOBILEEYESERVER_GEOM  -e bash -l -c "

CMD="$EXEC --name $MOBEYESERVER_NAME --file $CONFIG_PATH/$CONFIG_FILE $*"
# 
################################################################################

case "$scriptcmd" in
    kill)
		#kill_process $MACHINE ${EXEC##*/}
		stop_process $NAME $RUNNAME $MACHINE
		;;
    stop)
		stop_process $NAME $RUNNAME $MACHINE
		;;
	start)
		#start_process $RUNNAME $MACHINE "$CMD" "none" &
		start_process $RUNNAME $MACHINE "$CMD" "$XTERM_WRAP" &
		;;
	connect)
		#yarp wait $M_PORT
		#yarp wait $SOUNDSERVER_PORT_IN
		#yarp connect $PORTAUDIO_PORT $SOUNDSERVER_PORT_IN
		;;
    *)
		echo "usage: ${0##*/} [start|stop|connect|kill]"
		;;
esac
