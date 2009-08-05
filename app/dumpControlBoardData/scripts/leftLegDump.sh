#!/bin/bash

source ../conf/config.sh
APP="$ICUB_ROOT$APPLICATION_PATH"
#echo $APP

list=`grep "dataToDump" $APP/conf/$CONTROL_BOARD_DUMPER_LEFTLEG_FILE | tr -d \( | tr -d \)`
#echo $list

count=1;
for a in $list; do
    if [ $count -gt 1 ]
    then
	#echo "switching sequence $a..."

	#a dumper for each opened data
	command="$DUMPER_LEFTLEG_EXECUTABLE --name $DUMPER_LEFTLEG_NAME/$a"
	tag=dumperLeftLeg_$a
	node=$NODE_DUMPERS

	./impl_start.sh $1 $node $tag "$command"	
    fi;
    count=`expr $count + 1`
done


