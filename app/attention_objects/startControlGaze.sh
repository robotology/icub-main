#!/bin/bash

source ./config.sh

controlGaze2 --name $ROBOT/controlGaze2 --file $ICUB_ROOT/app/attention_objects/conf/icubControlGaze2.ini --motorboard /controlboard --appPath $ICUB_DIR/app/attention_objects --configCamera icubEyes.ini | rdin --name $ROBOT/controlGaze2 



