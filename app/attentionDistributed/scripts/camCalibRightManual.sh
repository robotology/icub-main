#!/bin/bash

source ./config.sh

bash -c "camCalib --name $CAMCALIB_RIGHT_NAME --file $APPLICATION_PATH/conf/$CAMCALIB_RIGHT_CONFIG_FILE --group $CAMCALIB_RIGHT_CONFIG_GROUP | rdin --name $CAMCALIB_RIGHT_NAME"

