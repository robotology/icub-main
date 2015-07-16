#!/bin/bash -l

### Simple startup script for "yarp server".
# nat

echo "Parameters: $1 $2 $3"

ID=/`uname -n`

if [ "k$YARP_DIR" = "k" ];
then
	YARP_DIR=/usr
fi

case "$1" in
	start)
	        if [ $# -eq 1 ];
		    then
		        echo "Starting up yarp server"
			yarpserver3 --portdb /tmp/ports.db --subdb /tmp/subs.db >/dev/null 2>&1 &
			echo "done!"
		elif [ $# -eq 2 ];
		then
			case "$2" in
				yarpserver)
					echo "Starting up yarp server"
					yarpserver >/dev/null 2>&1 &
					echo "done!"
					;;
				ros) # ros option and yarpserver3 is the default
					echo "Starting up yarp server"
					yarpserver3 --portdb /tmp/ports.db --subdb /tmp/subs.db --ros >/dev/null 2>&1 &
					echo "done!"
					;;
				*) #yarpserver3 is the default
					echo "Starting up yarp server"
					yarpserver3 --portdb /tmp/ports.db --subdb /tmp/subs.db >/dev/null 2>&1 &
					echo "done!"
					;;
			esac
		elif [ "$# -eq 3" -a "$3 = "ros"" ];
		    then
			case "$2" in
				yarpserver)
					echo "Starting up yarp server"
					yarpserver --ros >/dev/null 2>&1 &
					echo "done!"
					;;
				yarpserver3) #yarpserver3 is the default
					echo "Starting up yarp server"
					yarpserver3 --portdb /tmp/ports.db --subdb /tmp/subs.db >/dev/null --ros 2>&1 &
					echo "done!"
					;;
			esac
		fi
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
