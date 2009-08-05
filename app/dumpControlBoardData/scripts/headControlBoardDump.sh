#!/bin/bash

source ../conf/config.sh

command="$CONTROL_BOARD_DUMPER_HEAD_EXECUTABLE --from $CONTROL_BOARD_DUMPER_HEAD_FILE"
tag=controlBoardDumperHead
node=$NODE_HEAD

./impl_start.sh $1 $node $tag "$command"