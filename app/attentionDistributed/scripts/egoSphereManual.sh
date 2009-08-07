#!/bin/bash

source ./config.sh

bash -c "egoSphere --name $EGOSPHERE_NAME --file $APPLICATION_PATH/conf/$EGOSPHERE_CONFIG_FILE --group $EGOSPHERE_CONFIG_GROUP --controlboard $NAME_CONTROLBOARD | rdin --name $EGOSPHERE_NAME"
