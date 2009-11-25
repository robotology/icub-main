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
LOGDIR=/usr/local/src/robot/iCub/app/ihaNew/logs/
fi
echo "Using Log Directory : $LOGDIR"
mkdir -p $LOGDIR
ts=`date +%y%m%d-%H%M%S`

LOGFILE=$LOGDIR/$ts-ds.log
echo "Logfile : $LOGFILE"

################################################################################
# 
RUNNAME=datastore
MACHINE_NAME=$DATASTORE_MACHINE
MACHINE=${!MACHINE_NAME}
EXEC=ihaNewExperienceMetricSpace
NAME=$DATASTORE_NAME
CONFIG_FILE=ihaExperienceMetricSpace.ini

# Set the paths for this machine (to be deprecated when ResourceFinder is finalized)
CNAME=$`echo $MACHINE_NAME"_CONFIG_PATH"`
CONFIG_PATH=`eval echo $CNAME`

XTERM_WRAP="xterm -fg white -bg black -title exp_metric_space -geometry $DATASTORE_GEOM  -e bash -l -c "

#CMD="$EXEC --name $DATASTORE_NAME --file $CONFIG_PATH/$CONFIG_FILE --connect_to_sensors $SMI_INTERFACE_SENSORS_OUT $*"
#CMD="$EXEC --name $DATASTORE_NAME --file $CONFIG_PATH/$CONFIG_FILE --connect_to_sensors $SMI_INTERFACE_SENSORS_OUT --save $LOGDIR/DSsave-$ts.txt $*"
CMD="$EXEC --name $DATASTORE_NAME --file $CONFIG_PATH/$CONFIG_FILE --connect_to_sensors $DYNAMICS_PORT_REWARD_OUT --save $LOGDIR/DSsave-$ts.txt $*"
#CMD="$EXEC --name $DATASTORE_NAME --file $CONFIG_PATH/$CONFIG_FILE --save $LOGDIR/DSsave-$ts.txt $*"
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
		start_process $RUNNAME $MACHINE "$CMD" "none" $LOGFILE &
		;;
	start)
		start_process $RUNNAME $MACHINE "$CMD" "$XTERM_WRAP" $LOGFILE &
		;;
    *)
		echo "usage: ${0##*/} [start|stop|kill]"
		;;
esac
