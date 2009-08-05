#!/bin/bash

source ./config.sh

bash -c "controlGaze2 --configCamera $CAMCALIB_RIGHT_CONFIG_FILE --name $CONTROLGAZE_NAME --file $APPLICATION_PATH/conf/$CONTROLGAZE_CONFIG_FILE --appPath $APPLICATION_PATH --motorboard $NAME_CONTROLBOARD | rdin --name $CONTROLGAZE_NAME"
