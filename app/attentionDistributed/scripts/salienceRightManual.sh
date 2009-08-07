#!/bin/bash

source ./config.sh

bash -c "salience --name $SALIENCE_RIGHT_NAME --file $APPLICATION_PATH/conf/$SALIENCE_RIGHT_CONFIG_FILE --group $SALIENCE_RIGHT_CONFIG_GROUP | rdin --name $SALIENCE_RIGHT_NAME"
