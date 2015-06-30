#!/bin/bash -l

### Simple startup script for "yarp server".
# nat

echo "Parameters: $1 $2"

ID=/`uname -n`

if [ "k$YARP_DIR" = "k" ];
then
	YARP_DIR=/usr
fi

case "$1" in
	start)
		case "$2" in
			yarpserver)
				echo "Starting up yarp server"
				yarpserver >/dev/null 2>&1 &
				echo "done!"
				;;
			*) #yarpserver3 is the default
				echo "Starting up yarp server"
				yarpserver3 --portdb /tmp/ports.db --subdb /tmp/subs.db >/dev/null 2>&1 &
				echo "done!"
				;;
		esac
		;;
	stop)
		echo "Stopping yarp server"
		killall yarpserver3
		killall yarpserver
		;;

	kill)
		echo "Killing yarp server"
		killall -9 yarpserver3
		killall -9 yarpserver
		;;
	*)
		echo "start stop kill"
		;;
esac
