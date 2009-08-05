#!/bin/bash

# Copyright (C) 2007 Jonas Ruesch
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# pass 'connect' or 'disconnect' in $1 argument to this script

source ../conf/config.sh
APP="$ICUB_ROOT$APPLICATION_PATH"
#echo $APP

for a in $PARTS_TO_DUMP; do
    case $a in
	torso)
	    list=`grep "dataToDump" $APP/conf/$CONTROL_BOARD_DUMPER_TORSO_FILE | tr -d \( | tr -d \)`
	    #echo $list

	    count=1;
	    for a in $list; do
		if [ $count -gt 1 ]
		then
	            #echo "switching sequence $a..."
		    yarp $1 $CONTROL_BOARD_DUMPER_TORSO_NAME/$a $DUMPER_TORSO_NAME/$a
		fi;
		count=`expr $count + 1`
	    done
	    ;;
	head)
	    list=`grep "dataToDump" $APP/conf/$CONTROL_BOARD_DUMPER_HEAD_FILE | tr -d \( | tr -d \)`
	    #echo $list

	    count=1;
	    for a in $list; do
		if [ $count -gt 1 ]
		then
	            #echo "switching sequence $a..."
		    yarp $1 $CONTROL_BOARD_DUMPER_HEAD_NAME/$a $DUMPER_HEAD_NAME/$a
		fi;
		count=`expr $count + 1`
	    done
	    ;;
	left_arm)
	    list=`grep "dataToDump" $APP/conf/$CONTROL_BOARD_DUMPER_LEFTARM_FILE | tr -d \( | tr -d \)`
	    #echo $list

	    count=1;
	    for a in $list; do
		if [ $count -gt 1 ]
		then
	            #echo "switching sequence $a..."
		    yarp $1 $CONTROL_BOARD_DUMPER_LEFTARM_NAME/$a $DUMPER_LEFTARM_NAME/$a
		fi;
		count=`expr $count + 1`
	    done
	    ;;
	right_arm)
	    list=`grep "dataToDump" $APP/conf/$CONTROL_BOARD_DUMPER_RIGHTARM_FILE | tr -d \( | tr -d \)`
	    #echo $list

	    count=1;
	    for a in $list; do
		if [ $count -gt 1 ]
		then
	            #echo "switching sequence $a..."
		    yarp $1 $CONTROL_BOARD_DUMPER_RIGHTARM_NAME/$a $DUMPER_RIGHTARM_NAME/$a
			#echo "Trying to connect"
			#echo $CONTROL_BOARD_DUMPER_RIGHTARM_NAME/$a
			#echo $DUMPER_RIGHTARM_NAME/$a
		fi;
		count=`expr $count + 1`
	    done
	    ;;
	left_leg)
	    list=`grep "dataToDump" $APP/conf/$CONTROL_BOARD_DUMPER_LEFTLEG_FILE | tr -d \( | tr -d \)`
	    #echo $list

	    count=1;
	    for a in $list; do
		if [ $count -gt 1 ]
		then
	            #echo "switching sequence $a..."
		    yarp $1 $CONTROL_BOARD_DUMPER_LEFTLEG_NAME/$a $DUMPER_LEFTLEG_NAME/$a
		fi;
		count=`expr $count + 1`
	    done
	    ;;
	right_leg)
	    list=`grep "dataToDump" $APP/conf/$CONTROL_BOARD_DUMPER_RIGHTLEG_FILE | tr -d \( | tr -d \)`
	    #echo $list

	    count=1;
	    for a in $list; do
		if [ $count -gt 1 ]
		then
	            #echo "switching sequence $a..."
		    yarp $1 $CONTROL_BOARD_DUMPER_RIGHTLEG_NAME/$a $DUMPER_RIGHTLEG_NAME/$a
		fi;
		count=`expr $count + 1`
	    done
	    ;;
    esac
done
