#!/bin/sh

# Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
# Author: Assif Mirza 
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Robot Name
NAME_ROBOT=icub
NAME_APP=iha

## Robot Ports #################################################

DRAGONFLY_LEFT_NAME=/$NAME_ROBOT/cam/left
DRAGONFLY_RIGHT_NAME=/$NAME_ROBOT/cam/right
CAMCALIB_LEFT_NAME=/$NAME_ROBOT/camcalib/left
CAMCALIB_LEFT_PORT_IMAGE=$CAMCALIB_LEFT_NAME/image
CAMCALIB_RIGHT_NAME=/$NAME_ROBOT/camcalib/right
CAMCALIB_RIGHT_PORT_IMAGE=$CAMCALIB_RIGHT_NAME/image

# generic pointers
# To be used by anything that doesn't care whether
# it is a dragonfly or whatever
NAME_SOURCE_IMAGE_LEFTEYE=$DRAGONFLY_LEFT_NAME
NAME_SOURCE_IMAGE_RIGHTEYE=$DRAGONFLY_RIGHT_NAME

# Set a single variable to point to the camera
# so we don't have to specify left/right in scripts
CAMERA=$DRAGONFLY_LEFT_NAME


VIEWER_CAM_LEFT_PORT=/view/cam/left
VIEWER_FACEDETECT_PORT=/view/facedetect


## MODULE BASE NAMES ###########################################

FACEDETECT_NAME=/$NAME_APP/fd
PORTAUDIO_NAME=/$NAME_APP/pa
SOUNDSERVER_NAME=/$NAME_APP/sound
DYNAMICS_NAME=/$NAME_APP/dynamics
CONTROLLER_NAME=/$NAME_APP/controller
DATASTORE_NAME=/$NAME_APP/ds
ACTIONSEL_NAME=/$NAME_APP/as
SMI_INTERFACE_NAME=/$NAME_APP/sm
MONITOR_NAME=/$NAME_APP/status
SFW_NAME=/$NAME_APP/sfw


## MODULE PORTS ################################################

FACEDETECT_PORT_OUT=$FACEDETECT_NAME/facedetect:out
FACEDETECT_PORT_DETECT=$FACEDETECT_NAME/facedetect:coords
FACEDETECT_PORT_IN=$FACEDETECT_NAME/facedetect:in

PORTAUDIO_PORT=/$NAME_APP/sound_grabber

SOUNDSERVER_PORT_IN=$SOUNDSERVER_NAME/sndsensor:in
SOUNDSERVER_PORT_OUT=$SOUNDSERVER_NAME/sndsensor:out

DYNAMICS_PORT_SOUND_IN=$DYNAMICS_NAME/soundsensor:in
DYNAMICS_PORT_FACE_IN=$DYNAMICS_NAME/coords:in
DYNAMICS_PORT_REWARD_OUT=$DYNAMICS_NAME/reward:out

CONTROLLER_ACTION_CMD=$CONTROLLER_NAME/action:cmd
CONTROLLER_ENCODERS_OUT=$CONTROLLER_NAME/encoders:out

EXPRESSION_RAW_PORT=/$NAME_ROBOT/face/raw/in

DATASTORE_DIST_OUT=$DATASTORE_NAME/currdist:out
DATASTORE_STATUS_OUT=$DATASTORE_NAME/status:out

ACTIONSEL_STATUS_OUT=$ACTIONSEL_NAME/status:out

SMI_INTERFACE_SENSORS_OUT=$SMI_INTERFACE_NAME/sensor:out
SMI_INTERFACE_STATUS_OUT=$SMI_INTERFACE_NAME/status:out

MONITOR_OUT=$MONITOR_NAME/monitor:out
MONITOR_IN=$MONITOR_NAME/monitor:in

