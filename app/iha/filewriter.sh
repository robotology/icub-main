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
RUNNAME=sfw
EXEC=ihaSensorsFileWriter
MACHINE_NAME=$SFW_MACHINE
MACHINE=${!MACHINE_NAME}
NAME=$SFW_NAME
CONFIG_FILE=ihaSensorsFileWriter.ini
ACTIONDEFS_FILE=iha_actiondefs.ini
SEQUENCE_DIR=sequences

# Set the paths for this machine (to be deprecated when ResourceFinder is finalized)
CNAME=$`echo $MACHINE_NAME"_CONFIG_PATH"`
CONFIG_PATH=`eval echo $CNAME`

CMD="$EXEC --file $CONFIG_PATH/$CONFIG_FILE --connect_to_sensors $SMI_INTERFACE_SENSORS_OUT --connect_to_image $CAMERA $*"
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
		start_process $RUNNAME $MACHINE "$CMD" "none" &
		;;
    *)
		echo "usage: ${0##*/} [start|stop|kill]"
		;;
esac
