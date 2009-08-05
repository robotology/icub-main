#!/bin/bash

# Copyright (C) 2007 Alex Bernardino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# pass 'connect' or 'disconnect' in $1 argument to this script

source ./config.sh

#camera left -> camCalib
yarp $1 $NAME_SOURCE_IMAGE_LEFTEYE $CAMCALIB_LEFT_PORT_IMAGE
#camCalib -> viewer
yarp $1 $CAMCALIB_LEFT_PORT_IMAGE $VIEWER_CAMCALIB_LEFT_PORT
#camCalib left -> facedetect
yarp $1 $CAMCALIB_LEFT_PORT_IMAGE $FACEDETECT_PORT_IN
#camCalib left -> histtracker
yarp $1 $CAMCALIB_LEFT_PORT_IMAGE $HISTTRACKER_PORT_IN
#facedetect -> histtracker
yarp $1 $FACEDETECT_PORT_DETECT $HISTTRACKER_PORT_DETECT
#facedetect -> viewer
yarp $1 $FACEDETECT_PORT_OUT $VIEWER_FACEDETECT_PORT
#histtrack -> viewer
yarp $1 $HISTTRACKER_PORT_OUT $VIEWER_HISTTRACKER_PORT
#histtrack ->controlgaze
yarp $1 $HISTTRACKER_PORT_TRACK $CONTROLGAZE_PORT_VEL
