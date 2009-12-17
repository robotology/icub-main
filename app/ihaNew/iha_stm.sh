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
RUNNAME=stm
MACHINE_NAME=$STM_MACHINE
MACHINE=${!MACHINE_NAME}
EXEC=ihaNewShortTermMemory
NAME=$STM_NAME
CONFIG_FILE=ihaShortTermMemory.ini

# Set the paths for this machine (to be deprecated when ResourceFinder is finalized)
CNAME=$`echo $MACHINE_NAME"_CONFIG_PATH"`
CONFIG_PATH=`eval echo $CNAME`

XTERM_WRAP="xterm -hold -fg white -bg black -title stm -geometry $STM_GEOM  -e bash -l -c "

# Command to start motivation stm without making connections to face/sound ports
CMD="$EXEC --name $STM_NAME --file $CONFIG_PATH/$CONFIG_FILE --connect_to_data $SMI_INTERFACE_MEMSENSORS_OUT $*"
# 
################################################################################


case "$scriptcmd" in
    kill)
		#kill_process $MACHINE ${EXEC##*/}
		stop_process $NAME $RUNNAME $MACHINE
		;;
    stop)
	        yarp disconnect $SMI_INTERFACE_MEMSENSORS_OUT $STM_DATA_IN
		stop_process $NAME $RUNNAME $MACHINE
		;;
	start_noxterm)
		start_process $RUNNAME $MACHINE "$CMD" "none" &
		;;
	start)
		start_process $RUNNAME $MACHINE "$CMD" "$XTERM_WRAP" &
		;;
	connect)
		yarp wait $STM_PORT_DATA_IN
		yarp wait $SMI_INTERFACE_MEMSENSORS_OUT
		yarp wait $STM_SCORE_OUT
	        yarp connect $SMI_INTERFACE_MEMSENSORS_OUT $STM_DATA_IN
		;;
	disconnect)
	       yarp disconnect $SMI_INTERFACE_MEMSENSORS_OUT $STM_DATA_IN
		;;
    *)
		echo "usage: ${0##*/} [start|stop|kill|connect|disconnect]"
		;;
esac
