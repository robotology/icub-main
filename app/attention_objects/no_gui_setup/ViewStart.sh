killall yarpview
sleep 1
killall -9 yarpview


source ./config.sh

yarpview $VIEW_CALIB_LEFT >& viewCamCalibLeft.output &
yarp wait $VIEW_CALIB_LEFT

yarpview $VIEW_CALIB_RIGHT >& viewCamCalibRight.output &
yarp wait $VIEW_CALIB_RIGHT

yarpview $VIEW_SIFT >& viewSift.output &
yarp wait $VIEW_SIFT

yarpview $VIEW_EGO >& viewEgo.output &
yarp wait $VIEW_EGO

sleep 1

yarp connect $ROBOT/camCalib/left/out $VIEW_CALIB_LEFT
yarp connect $ROBOT/camCalib/right/out $VIEW_CALIB_RIGHT
yarp connect $ROBOT/sift/left $VIEW_SIFT
yarp connect $ROBOT/egoSphere/map_out $VIEW_EGO
