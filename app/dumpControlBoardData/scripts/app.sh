#!/bin/sh

# Start the dump
# 10/1/08, Francesco Nori

. ../conf/config.sh

#echo $NAME_ROBOT
#echo $PARTS_TO_DUMP

case $1 in
    start)
	for a in $PARTS_TO_DUMP; do
	    echo "Now I can begin launching the application"
	    case $a in
		head)
		    ./headControlBoardDump.sh start
		    ./headDump.sh start
		    ;;
		torso)
 		    ./torsoControlBoardDump.sh start
 		    ./torsoDump.sh start
		    ;;
		left_arm)
 		    ./leftArmControlBoardDump.sh start
 		    ./leftArmDump.sh start
		    ;;
		right_arm)
 		    ./rightArmControlBoardDump.sh start
 		    ./rightArmDump.sh start
		    ;;
		left_leg)
 		    ./leftLegControlBoardDump.sh start
 		    ./leftLegDump.sh start
		    ;;
		right_leg)
 		    ./rightLegControlBoardDump.sh start
 		    ./rightLegDump.sh start
		    ;;
	    esac
	done
	;;
    connect)
	echo "Connecting the application"
 	./connect.sh
 	;;
    stop)
	./disconnect.sh
	echo "Stopping the application"
	for a in $PARTS_TO_DUMP; do		    
	    case $a in
		head)
		    ./headDump.sh stop
 		    ./headControlBoardDump.sh stop
		    ;;
		torso)
 		    ./torsoDump.sh stop
 		    ./torsoControlBoardDump.sh stop
		    ;;
		left_arm)
 		    ./leftArmControlBoardDump.sh stop
 		    ./leftArmDump.sh stop
		    ;;
		right_arm)
 		    ./rightArmControlBoardDump.sh stop
 		    ./rightArmDump.sh stop
		    ;;
		left_leg)
 		    ./leftLegControlBoardDump.sh stop
 		    ./leftLegDump.sh stop
		    ;;
		right_leg)
 		    ./rightLegControlBoardDump.sh stop
 		    ./rightLegDump.sh stop
		    ;;
	    esac
	done
 	;;
    disconnect)
	./disconnect.sh
 	;;
    *)
 	echo "Start, stop, connect or disconnect"
 	;;
esac


