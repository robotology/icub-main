#!/bin/bash

source ./config_ports.sh

case "$1" in
    stop)
		killall yarpview
		;;
	start)
		# viewer cam left
		yarpview --name $VIEWER_CAM_LEFT_PORT --PosX 976 --PosY 0 &

		# viewer face detector
		yarpview --name $VIEWER_FACEDETECT_PORT --PosX 970 --PosY 800 &
		;;
	start_fdonly)
		# viewer face detector
		yarpview --name $VIEWER_FACEDETECT_PORT --PosX 2200 --PosY 800 &
		;;
	connect)
		# cam first
		# wait for ports and then connect
		yarp wait $VIEWER_CAM_LEFT_PORT
		yarp wait $CAMERA
		yarp connect $CAMERA $VIEWER_CAM_LEFT_PORT 

		# facedetect
		yarp wait $VIEWER_FACEDETECT_PORT
		# wait for the facedetector ports
		yarp wait $FACEDETECT_PORT_OUT
		# connect the facedetector output to its viewer
		yarp connect $FACEDETECT_PORT_OUT $VIEWER_FACEDETECT_PORT 
		;;
	connect_fdonly)
		# wait for ports and then connect
		yarp wait $CAMERA

		# facedetect
		yarp wait $VIEWER_FACEDETECT_PORT
		# wait for the facedetector ports
		yarp wait $FACEDETECT_PORT_OUT
		# connect the facedetector output to its viewer
		yarp connect $FACEDETECT_PORT_OUT $VIEWER_FACEDETECT_PORT 
		;;
    *)
		echo "usage: ${0##*/} [start|stop|connect|start_fdonly|connect_fdonly]"
		;;
esac
