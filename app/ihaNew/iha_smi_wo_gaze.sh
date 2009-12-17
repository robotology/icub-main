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

LOGFILE=$LOGDIR/$ts-smi.log
echo "Logfile : $LOGFILE"

################################################################################
# 
RUNNAME=smi
MACHINE_NAME=$SMI_INTERFACE_MACHINE
MACHINE=${!MACHINE_NAME}
EXEC=ihaNewSensorMotorInterface
NAME=$SMI_INTERFACE_NAME
CONFIG_FILE=ihaSensorMotorInterface.ini

# Set the paths for this machine (to be deprecated when ResourceFinder is finalized)
CNAME=$`echo $MACHINE_NAME"_CONFIG_PATH"`
CONFIG_PATH=`eval echo $CNAME`

XTERM_WRAP="xterm -hold -fg white -bg darkslategray -title iha_smi -geometry $SMI_GEOM  -e bash -l -c "


#CMD="$EXEC --name $SMI_INTERFACE_NAME --file $CONFIG_PATH/$SMI_INTERFACE_CONFIG_FILE  --connect_to_image $CAMERA --connect_to_face_coords $FACEDETECT_PORT_DETECT --connect_to_soundsensor $SOUNDSERVER_PORT_OUT --connect_to_encoders $CONTROLLER_ENCODERS_OUT --gaze_input FALSE $*"
CMD="$EXEC --name $SMI_INTERFACE_NAME --file $CONFIG_PATH/$SMI_INTERFACE_CONFIG_FILE  --connect_to_image $CAMERA --connect_to_face_coords $FACEDETECT_PORT_DETECT --connect_to_soundsensor $SOUNDSERVER_PORT_OUT --connect_to_encoders $CONTROLLER_ENCODERS_OUT --connect_to_beat $AUDIOANALYSER_PORT_OUT --gaze_input FALSE --beat_input TRUE $*"

# 
################################################################################

case "$scriptcmd" in
	kill)
		#kill_process $MACHINE ${EXEC##*/}
		stop_process $NAME $RUNNAME $MACHINE
		;;
	stop)
		yarp disconnect $FACEDETECT_PORT_DETECT $SMI_INTERFACE_FACE_IN
		#yarp disconnect $DYNAMICS_PORT_REWARD_OUT $SMI_INTERFACE_NAME/reward:in
		yarp disconnect $SOUNDSERVER_PORT_OUT $SMI_INTERFACE_SOUND_IN
		yarp disconnect $CONTROLLER_ENCODERS_OUT $SMI_INTERFACE_ENCODERS_IN
		stop_process $NAME $RUNNAME $MACHINE
		;;

        connect)
                yarp wait $SMI_INTERFACE_IMAGE_IN
		yarp wait $FACEDETECT_PORT_DETECT 
                yarp wait $SMI_INTERFACE_FACE_IN
		yarp wait $SOUNDSERVER_PORT_OUT 
		#yarp wait $SMI_INTERFACE_SOUND_IN
		yarp connect $FACEDETECT_PORT_DETECT $SMI_INTERFACE_FACE_IN
		#yarp disconnect $DYNAMICS_PORT_REWARD_OUT $SMI_INTERFACE_NAME/reward:in
		yarp connect $SOUNDSERVER_PORT_OUT $SMI_INTERFACE_SOUND_IN

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
