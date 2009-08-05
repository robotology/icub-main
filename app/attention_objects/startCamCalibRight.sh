#!/bin/bash

source ./config.sh

camCalib --name $ROBOT/camCalib/right --file $ICUB_ROOT/app/attention_objects/conf/icubEyes.ini --group CAMERA_CALIBRATION_RIGHT | rdin --name $ROBOT/camCalib/right

