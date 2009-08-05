#!/bin/bash

appDir=${ICUB_ROBOTNAME:-default}

source ../../$appDir/scripts/cluster-config.sh

case $1 in
    check)
	echo "Checking nameserver"
	if yarp exists $NAMESPACE; then
	    echo "Ok, namesapce $NAMESPACE available"
	else
	    echo "No $NAMESPACE available check nameserver"
	    exit
	fi
	
	echo "Now checking yarp run servers"

	for a in $CLUSTERS
	  do
	  echo "Going to ping /$a"
	  if yarp exists /$a; then
	      echo "Ok, yarp run found on $a"
	  else
	      echo "No yarp run found on $a"
	  fi
	done
	;;

    start)
	echo "Checking nameserver"
	if yarp exists $NAMESPACE; then
	    echo "Ok, $NAMESPACE available"
	else
	    echo "Nameserver $NAMESPACE not found"
	    echo "You need to start it manually on $NODE_NAMESERVER"
	    echo "Open a console and type 'yarpserver &'"
	    exit
	fi
	
	echo "Now checking yarp run servers"

	for a in $CLUSTERS
	  do
	  echo "Going to ping /$a"
	  if yarp exists /$a; then
	      echo "Ok, yarp run found on $a"
	  else
	      echo "Starting yarp run on $a"
	      ssh -f icub@$a "/etc/init.d/yarprun start"
	  fi
	done
	;;
    stop)
	echo "Now stopping yarp run servers"
	for a in $CLUSTERS
	  do
	  echo "Going to ping /$a"
	  if yarp exists /$a; then
	      ssh -f icub@$a "/etc/init.d/yarprun stop"
	  else
	      echo "yarp run not running on $a, skipping it"
	  fi
	done
	;;

    killall)
	for a in $CLUSTERS
	  do
	  echo "Going to force a killall yarp on /$a"
	  ssh -f icub@$a "killall yarp"
	done
	;;
    *)
	echo "check: checks if yarp run is running"
	echo "start: starts yarp run on all machines"
	echo "stop: stops yarp run on all machines"
	echo "killall: forces a kill of yarp run on all machines"
	;;

esac    

