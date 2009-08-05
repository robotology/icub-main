#!/bin/bash

source ../conf/config.sh

command="$CONTROL_BOARD_DUMPER_TORSO_EXECUTABLE --from $CONTROL_BOARD_DUMPER_TORSO_FILE"
tag=controlBoardDumperTorso
node=$NODE_TORSO

./impl_start.sh $1 $node $tag "$command"
