

#!/bin/bash

MYSHELL="bash -c"

case "$1" in
	kill)
		killall -9 velImpControl
		killall -9 velocityControl
		;;
    	stop)
		killall velImpControl
		killall velocityControl
		;;
 	start)
		$MYSHELL "velImpControl --part left_arm --njoints 4"&
		sleep 1
		$MYSHELL "velImpControl --part right_arm --njoints 4"&
		sleep 1
		$MYSHELL "velImpControl --part left_leg --njoints 4"&
		sleep 1
		$MYSHELL "velImpControl --part right_leg --njoints 4"&
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
