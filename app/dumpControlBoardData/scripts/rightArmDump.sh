#!/bin/bash

source ../conf/config.sh
APP="$ICUB_ROOT$APPLICATION_PATH"
#echo $APP

list=`grep "dataToDump" $APP/conf/$CONTROL_BOARD_DUMPER_RIGHTARM_FILE | tr -d \( | tr -d \)`
#echo $list

count=1;
for a in $list; do
    if [ $count -gt 1 ]
    then
	#echo "switching sequence $a..."

	#a dumper for each opened data
	command="$DUMPER_RIGHTARM_EXECUTABLE --name $DUMPER_RIGHTARM_NAME/$a"
	tag=dumperRightArm.$a
	node=$NODE_DUMPERS

	./impl_start.sh $1 $node $tag "$command"	
    fi;
    count=`expr $count + 1`
done


