#!/bin/bash

source ../conf/config.sh

command="$CONTROL_BOARD_DUMPER_LEFTARM_EXECUTABLE --from $CONTROL_BOARD_DUMPER_LEFTARM_FILE"
tag=controlBoardDumperLeftArm
node=$NODE_LEFTARM

./impl_start.sh $1 $node $tag "$command"