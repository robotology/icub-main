#!/bin/bash

source ./config.sh

bash -c "attentionSelection --name $ATTENTIONSELECTION_NAME --file $APPLICATION_PATH/conf/$ATTENTIONSELECTION_CONFIG_FILE --group $ATTENTIONSELECTION_CONFIG_GROUP --remoteEgoSphere $EGOSPHERE_PORT_CONF | rdin --name $ATTENTIONSELECTION_NAME"
