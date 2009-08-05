#!/bin/bash

source ./config.sh

attentionSelection --name $ROBOT/attentionSelection --file $ICUB_ROOT/app/attention_objects/conf/icubAttentionSelection.ini | rdin --name $ROBOT/attentionSelection

