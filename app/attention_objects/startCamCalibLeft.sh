#!/bin/bash

source ./config.sh

camCalib --name $ROBOT/camCalib/left --file $ICUB_ROOT/app/attention_objects/conf/icubEyes.ini --group CAMERA_CALIBRATION_LEFT | rdin --name $ROBOT/camCalib/left
