#!/bin/bash

source ../conf/config.sh

command="$CONTROL_BOARD_DUMPER_RIGHTLEG_EXECUTABLE --from $CONTROL_BOARD_DUMPER_RIGHTLEG_FILE"
tag=controlBoardDumperRightLeg
node=$NODE_RIGHTLEG

./impl_start.sh $1 $node $tag "$command"