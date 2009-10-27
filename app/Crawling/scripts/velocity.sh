

#!/bin/bash

MYSHELL="bash -c"

case "$1" in
	kill)
		killall -9 velocityControl
		;;
    	stop)
		killall velocityControl
		;;
 	start)
		$MYSHELL "velocityControl --part left_arm"&
		sleep 1
		$MYSHELL "velocityControl --part right_arm"&
		sleep 1
		$MYSHELL "velocityControl --part left_leg"&
		sleep 1
		$MYSHELL "velocityControl --part right_leg"&
		sleep 1
		$MYSHELL "velocityControl --part torso"&
        sleep 1
		$MYSHELL "velocityControl --part head"&
		sleep 1
		;;

    *)
		echo "stop, start, kill"
		;;
esac
