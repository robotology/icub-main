#!/bin/bash

source ./config_machines.sh
source ./config_ports.sh
source ./start_stop_utils.sh

# Eat the first (command) parameter
# This allows you to pass extra parameters to the command executed (e.g. ./script.sh start --dbg 40)
scriptcmd=$1
shift

# Logfile
# if the environment variable is set already, then use that otherwise set a defa
ult
if [ ${#LOGDIR} -eq 0 ]
then
LOGDIR=/tmp/data/logs/
fi
echo "Using Log Directory : $LOGDIR"
mkdir -p $LOGDIR
ts=`date +%y%m%d-%H%M%S`

LOGFILE=$LOGDIR/$ts-control.log
echo "Logfile : $LOGFILE"



################################################################################
# 
RUNNAME=controller
EXEC=ihaNewIcubControl
MACHINE_NAME=$CONTROLLER_MACHINE
MACHINE=${!MACHINE_NAME}
NAME=$CONTROLLER_NAME
CONFIG_FILE=ihaIcubControl.ini
ACTIONDEFS_FILE=iha_actiondefs.ini
SEQUENCE_DIR=sequences

# Set the paths for this machine (to be deprecated when ResourceFinder is finalized)
CNAME=$`echo $MACHINE_NAME"_CONFIG_PATH"`
CONFIG_PATH=`eval echo $CNAME`

XTERM_WRAP="xterm -fg white -bg darkblue -title controller -geometry $CONTROLLER_GEOM -e bash -l -c "

CMD="$EXEC --name $CONTROLLER_NAME --robot $NAME_ROBOT --file $CONFIG_PATH/$CONFIG_FILE --action_defs $CONFIG_PATH/$ACTIONDEFS_FILE --sequence_dir $CONFIG_PATH/$SEQUENCE_DIR --speed_multiplier 3 --connect_to_expression $EXPRESSION_RAW_PORT $*"
#CMD="$EXEC --name $CONTROLLER_NAME --robot $NAME_ROBOT --file $CONFIG_PATH/$CONFIG_FILE --action_defs $CONFIG_PATH/$ACTIONDEFS_FILE --sequence_dir $CONFIG_PATH/$SEQUENCE_DIR --speed_multiplier 1 $*"
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
	start_noxterm)
		start_process $RUNNAME $MACHINE "$CMD" "none" &
		;;
	start)
		start_process $RUNNAME $MACHINE "$CMD" "$XTERM_WRAP" $LOGFILE &
		;;
    *)
		echo "usage: ${0##*/} [start|stop|kill]"
		;;
esac
