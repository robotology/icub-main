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
LOGDIR=/tmp/data/logs/
fi
echo "Using Log Directory : $LOGDIR"
mkdir -p $LOGDIR
ts=`date +%y%m%d-%H%M%S`

LOGFILE=$LOGDIR/$ts-sound.log
echo "Logfile : $LOGFILE"


################################################################################
# 
RUNNAME=soundserver
MACHINE_NAME=$SOUNDSERVER_MACHINE
MACHINE=${!MACHINE_NAME}
EXEC=ihaNewSoundSensor
NAME=$SOUNDSERVER_NAME
CONFIG_FILE=ihaSoundSensor.ini

# Set the paths for this machine (to be deprecated when ResourceFinder is finalized)
CNAME=$`echo $MACHINE_NAME"_CONFIG_PATH"`
CONFIG_PATH=`eval echo $CNAME`

XTERM_WRAP="xterm -fg white -bg black -title soundserver -geometry $SOUNDSERVER_GEOM  -e bash -l -c "

CMD="$EXEC --name $SOUNDSERVER_NAME --file $CONFIG_PATH/$CONFIG_FILE $*"
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
		#start_process $RUNNAME $MACHINE "$CMD" "$XTERM_WRAP" &
	        start_process $RUNNAME $MACHINE "$CMD" "$XTERM_WRAP" $LOGFILE &
		;;
	connect)
		yarp wait $PORTAUDIO_PORT
		yarp wait $SOUNDSERVER_PORT_IN
		yarp connect $PORTAUDIO_PORT $SOUNDSERVER_PORT_IN
		;;
    *)
		echo "usage: ${0##*/} [start|stop|connect|kill]"
		;;
esac
