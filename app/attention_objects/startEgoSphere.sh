#!/bin/bash

source ./config.sh

egoSphere --name $ROBOT/egoSphere --file $ICUB_ROOT/app/attention_objects/conf/icubEyes.ini --group EGOSPHERE | rdin --name $ROBOT/egoSphere

