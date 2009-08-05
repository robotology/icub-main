#!/bin/bash

killall siftObjectRepresentation

sleep 3

source ./config.sh

/$ICUB_DIR/src/siftObjectRepresentation/siftObjectRepresentation --name $ROBOT/sift --file $ICUB_EYES_INI >& sift.output &

yarp wait $ROBOT/sift/left
yarp wait $ROBOT/sift/positions

yarp connect $ROBOT/sift/left $VIEW_SIFT
yarp connect $ROBOT/camCalib/left/out $ROBOT/sift/left
yarp connect $ROBOT/camCalib/right/out $ROBOT/sift/right
yarp connect $ROBOT/sift/positions $ROBOT/egoSphere/mapObject/bottle_in

yarp connect $ROBOT/sift/sacPos /chica/controlGaze2/pos


