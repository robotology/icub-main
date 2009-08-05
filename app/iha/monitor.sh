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
RUNNAME=monitor
MACHINE_NAME=$MONITOR_MACHINE
MACHINE=${!MACHINE_NAME}
EXEC=ihaStatusMonitor
NAME=$MONITOR_NAME
CONFIG_FILE=ihaStatusMonitor.ini

# Set the paths for this machine (to be deprecated when ResourceFinder is finalized)
CNAME=$`echo $MACHINE_NAME"_CONFIG_PATH"`
CONFIG_PATH=`eval echo $CNAME`

CMD="$EXEC --name $MONITOR_NAME --file $CONFIG_PATH/$CONFIG_FILE $*"
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
	connect)
		yarp wait $DATASTORE_STATUS_OUT
		yarp connect $DATASTORE_STATUS_OUT $MONITOR_IN
		yarp wait $ACTIONSEL_STATUS_OUT
		yarp connect $ACTIONSEL_STATUS_OUT $MONITOR_IN
		yarp wait $SMI_INTERFACE_STATUS_OUT
		yarp connect $SMI_INTERFACE_STATUS_OUT $MONITOR_IN
		;;
    *)
		echo "usage: ${0##*/} [start|stop|kill|connect]"
		;;
esac
