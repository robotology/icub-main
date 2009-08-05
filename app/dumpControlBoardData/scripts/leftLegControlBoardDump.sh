#!/bin/bash

source ../conf/config.sh

command="$CONTROL_BOARD_DUMPER_LEFTLEG_EXECUTABLE --from $CONTROL_BOARD_DUMPER_LEFTLEG_FILE"
tag=controlBoardDumperLeftLeg
node=$NODE_LEFTLEG

./impl_start.sh $1 $node $tag "$command"