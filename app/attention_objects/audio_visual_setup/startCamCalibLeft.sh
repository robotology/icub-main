#!/bin/bash

source ./../config.sh

camCalib --name $ROBOT/camCalib/left --file $ICUB_DIR/app/attention_objects/conf/icubEyes.ini --group CAMERA_CALIBRATION_LEFT >& camCalib.output &

yarp wait $ROBOT/camCalib/left/in

yarp connect $LeftEye $ROBOT/camCalib/left/in

yarp connect $ROBOT/camCalib/left/out $CALIB_LEFT_VIEW
yarp connect $ROBOT/camCalib/left/out $ROBOT/sift/left