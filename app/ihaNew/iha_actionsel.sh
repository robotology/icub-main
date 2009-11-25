#!/bin/bash

source ./config_machines.sh
source ./config_ports.sh
source ./start_stop_utils.sh

# Eat the first (command) parameter
# This allows you to pass extra parameters to the command executed (e.g. ./script.sh start --dbg 40)
scriptcmd=$1
shift

# Logfile
# if the environment variable is set already, then use that otherwise set a default
if [ ${#LOGDIR} -eq 0 ]
then
LOGDIR=/usr/local/src/robot/iCub/app/ihaNew/logs
fi
echo "Using Log Directory : $LOGDIR"
mkdir -p $LOGDIR
ts=`date +%y%m%d-%H%M%S`

LOGFILE=$LOGDIR/$ts-ac.log
echo "Logfile : $LOGFILE"

################################################################################
# 
RUNNAME=actionsel
MACHINE_NAME=$ACTIONSEL_MACHINE
MACHINE=${!MACHINE_NAME}
EXEC=ihaNewActionSelection
NAME=$ACTIONSEL_NAME
CONFIG_FILE=ihaActionSelection.ini

# Set the paths for this machine (to be deprecated when ResourceFinder is finalized)
CNAME=$`echo $MACHINE_NAME"_CONFIG_PATH"`
CONFIG_PATH=`eval echo $CNAME`

XTERM_WRAP="xterm -fg white -bg black -title action_select -geometry $ACTIONSEL_GEOM  -e bash -l -c "

CMD="$EXEC  --name $ACTIONSEL_NAME --file $CONFIG_PATH/$CONFIG_FILE --connect_to_action $CONTROLLER_ACTION_CMD --connect_to_dist $DATASTORE_DIST_OUT --action_defs $CONFIG_PATH/iha_actiondefs.ini --sequence_dir $CONFIG_PATH/sequences $*"
#CMD="$EXEC  --name $ACTIONSEL_NAME --file $CONFIG_PATH/$CONFIG_FILE --connect_to_dist $DATASTORE_DIST_OUT --action_defs $CONFIG_PATH/iha_actiondefs.ini --sequence_dir $CONFIG_PATH/sequences $*"
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
		start_process $RUNNAME $MACHINE "$CMD" "$XTERM_WRAP" $LOGFILE &
		;;
    *)
		echo "usage: ${0##*/} [start|stop|kill]"
		;;
esac
