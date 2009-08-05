#!/bin/bash

source ./../config.sh

camCalib --name $ROBOT/camCalib/right --file $ICUB_DIR/app/attention_objects/conf/icubEyes.ini --group CAMERA_CALIBRATION_RIGHT >& camCalibRight.output &

yarp wait $ROBOT/camCalib/right/in

yarp connect $RightEye $ROBOT/camCalib/right/in

yarp connect $ROBOT/camCalib/right/out $CALIB_RIGHT_VIEW
yarp connect $ROBOT/camCalib/right/out $ROBOT/sift/right