#!/bin/bash

killall siftObjectRepresentation

source ./../config.sh

/$ICUB_DIR/src/siftObjectRepresentation/siftObjectRepresentation --name $ROBOT/sift --file $icubEyesIni >& sift.output &

yarp wait $ROBOT/sift/left
yarp wait $ROBOT/sift/positions

yarp connect $ROBOT/sift/left $siftView
yarp connect $ROBOT/camCalib/left/out $ROBOT/sift/left
yarp connect $ROBOT/camCalib/right/out $ROBOT/sift/right
yarp connect $ROBOT/sift/sacPos /icub/controlGaze/pos


