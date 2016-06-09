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
			ros) # ros option
				echo "Starting up yarp server with ros option"
				yarpserver --portdb :memory: --subdb :memory: --ros >/dev/null 2>&1 &
				echo "done!"
				;;
			*) #yarpserver without ros is the default
				echo "Starting up yarp server"
				yarpserver --portdb :memory: --subdb :memory: >/dev/null 2>&1 &
				echo "done!"
				;;
		esac
	        ;;
	stop)
		echo "Stopping yarp server"
		killall yarpserver
		;;

	kill)
		echo "Killing yarp server"
		killall -9 yarpserver
		;;
	*)
		echo "start stop kill"
		;;
esac
