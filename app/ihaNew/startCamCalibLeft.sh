#!/bin/bash

source ./config_ports.sh

# Module: Camera Calibration Left Eye
CAMCALIB_LEFT_EXECUTABLE=camCalib
CAMCALIB_LEFT_CONFIG_FILE=icubEyes.ini
CAMCALIB_LEFT_CONFIG_GROUP=CAMERA_CALIBRATION_LEFT


camCalib --name $CAMCALIB_LEFT_NAME --file $ICUB_ROOT/app/iha/conf/$CAMCALIB_LEFT_CONFIG_FILE --group $CAMCALIB_LEFT_CONFIG_GROUP
