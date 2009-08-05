#!/bin/bash

killall siftObjectRepresentation

source ./../config.sh

/$ICUB_DIR/src/siftObjectRepresentation/siftObjectRepresentation --name $ROBOT/sift --file $ICUB_DIR/app/attention_objects/conf/icubEyes.ini >& sift.output &

yarp wait $ROBOT/sift/left
yarp wait $ROBOT/sift/positions
yarp wait $ROBOT/sift/sacPos

yarp connect $ROBOT/sift/left /v
yarp connect $ROBOT/camCalib/left/out $ROBOT/sift/left
yarp connect $ROBOT/camCalib/right/out $ROBOT/sift/right
yarp connect $ROBOT/sift/sacPos /icub/controlGaze/pos


