#!/bin/bash

source ../conf/config.sh

command="$CONTROL_BOARD_DUMPER_RIGHTARM_EXECUTABLE --from $CONTROL_BOARD_DUMPER_RIGHTARM_FILE"
tag=controlBoardDumperRightArm
node=$NODE_RIGHTARM

./impl_start.sh $1 $node $tag "$command"