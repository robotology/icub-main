#!/bin/bash

source ./config_ports.sh

case "$1" in
        stop)
         	yarp disconnect /face/eyelids /icubSim/face/eyelids 
        	yarp disconnect /face/image/out /icubSim/texture
		yarp disconnect $OPENCV_NAME $CAM_SIM_PORT 
		yarp terminate $OPENCV_NAME/quit
		;;
	start)
		yarpdev --device opencv_grabber --name ${OPENCV_NAME} &
		yarpdev --device dv_grabber --name ${DVGRAB_NAME} --framerate 25 &
		;;
	connect)
		# cam first
		# wait for ports and then connect
		yarp wait $OPENCV_NAME
		yarp wait $CAM_SIM_PORT
		yarp connect $OPENCV_NAME $CAM_SIM_PORT 
		yarp connect /face/eyelids /icubSim/face/eyelids 
		yarp connect /face/image/out /icubSim/texture

		;;
        *)
		echo "usage: ${0##*/} [start|stop]"
		;;
esac
