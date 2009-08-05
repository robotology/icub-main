#!/bin/bash

source ./config.sh

YARP_Disparity_SiftCV --name $ROBOT/sift --file $ICUB_ROOT/app/attention_objects/conf/icubEyes.ini | rdin --name $ROBOT/sift


