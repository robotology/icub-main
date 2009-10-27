

#!/bin/bash

MYSHELL="bash -c"

case "$1" in
	kill)
		killall -9 CrawlGeneratorTorso
		;;
    	stop)
		killall CrawlGeneratorTorso
		;;
 	start)
		$MYSHELL "CrawlGeneratorTorso --part left_arm"&
		sleep 1
		$MYSHELL "CrawlGeneratorTorso --part right_arm"&
		sleep 1
		$MYSHELL "CrawlGeneratorTorso --part left_leg"&
		sleep 1
		$MYSHELL "CrawlGeneratorTorso --part right_leg"&
		sleep 1
		$MYSHELL "CrawlGeneratorTorso --part torso > output.txt" &
        sleep 1
		$MYSHELL "CrawlGeneratorTorso --part head"&
		sleep 1
		;;

    *)
		echo "stop, start, kill"
		;;
esac
