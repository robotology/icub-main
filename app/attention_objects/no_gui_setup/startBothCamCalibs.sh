#!/bin/bash

killall camCalib

source ./config.sh

$ICUB_ROOT/src/camCalib/camCalib --name $ROBOT/camCalib/left --file $ICUB_EYES_INI --group CAMERA_CALIBRATION_LEFT >& camCalibLeft.output &
yarp wait $ROBOT/camCalib/left/in
yarp connect /LeftEye $ROBOT/camCalib/left/in

$ICUB_ROOT/src/camCalib/camCalib --name $ROBOT/camCalib/right --file $ICUB_EYES_INI --group CAMERA_CALIBRATION_RIGHT >& camCalibRight.output &
yarp wait $ROBOT/camCalib/right/in
yarp connect /RightEye $ROBOT/camCalib/right/in

yarp connect $ROBOT/camCalib/left/out $ROBOT/sift/left
yarp connect $ROBOT/camCalib/right/out $ROBOT/sift/right

yarp connect $ROBOT/camCalib/left/out $VIEW_CALIB_LEFT
yarp connect $ROBOT/camCalib/right/out $VIEW_CALIB_RIGHT
