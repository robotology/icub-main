#!/bin/bash 
#
# Author: Assif Mirza 2008
# Copyright: RobotCub Consortium

# Three functions:

# start_process 
#  params: 
#     RUNNAME     e.g. "dynamics"
#     MACHINE     e.g. icub-b1
#     CMD         e.g. "exec --file my.ini"
#     XTERM_WRAP  e.g. "xterm -e bash -l -c "
#     LOGFILE     e.g. /tmp/logfile

# stop_process  NAME RUNNAME MACHINE
#
# removed for now --> kill_process MACHINE EXECNAME

function start_process () {
	runname=$1
	machine=$2
	cmd=$3
	xterm_wrap=$4
	logfile=$5

	if [ "X$logfile" == "X" ] || [ "X$logfile" == "Xnone" ]
	then
		logepithet=""
	else
		logepithet="(logfile: $logfile)"
		rm -f $logfile
	fi
	if [ "X$xterm_wrap" == "Xnone" ]
	then
		xterm_wrap="bash -c "
	fi

	echo "==================  $runname ======================"
	echo MACHINE $machine
	echo CMD $cmd 

	if [ "X$machine" != "Xlocal" ]
	then
		echo "Starting $runname using yarp run on /$machine $logepithet"

		yarp run --on /$machine --as $runname --cmd "$cmd" 

		if [ "X$logfile" == "X" ]
		then
			# echo the output to the calling terminal (stderr) 
			$xterm_wrap "ihaNewPortReader --port /$runname/stdout" &
		else
			# We can also write to the log
			$xterm_wrap "ihaNewPortReader --port /$runname/stdout --filename ${logfile##*/} --dir ${logfile%/*}" &
		fi

	else 
		echo "Starting $runname LOCAL $logepithet"

		if [ "X$xterm_wrap" == "X" ]
		then
			if [ "X$logfile" == "X" ]
			then
				echo Local Command $cmd
				eval $cmd
			else
				eval $cmd 2>&1 | tee $logfile
			fi
		else
			if [ "X$logfile" == "X" ]
			then
				$xterm_wrap "$cmd"
			else
				$xterm_wrap "$cmd 2>&1 | tee $logfile"
			fi
		fi

	fi

}

# stop_process
#
#  This will first try nicely to send a quit message to the process
#  If it won't die (and it is local!)  then it will have a go at killing 
#  the pid instead. This is no good if it is remote BTW
#  Finally, if the process was remote, it cleans up the local reader processes

function stop_process () {
	name=$1
	runname=$2
	machine=$3

	# local:
	if [ "X$machine" != "Xlocal" ]
	then
		# for proccesses started on remote machines 
		# use the yarp run process to kill it 

		echo "Ask yarp run to kill $runname"
		yarp run --on $machine --kill $runname

		# tidy up the logger process

		PID=`ps -ef | grep ihaNewPortReader | grep $runname | awk '{print $2}'`
		if [ ${#PID} -ne 0 ] 
		then
			kill $PID
		fi
		PID=`ps -ef | grep ihaNewPortReader | grep $runname | awk '{print $2}'`
		if [ ${#PID} -ne 0 ] 
		then
			kill -9 $PID
		fi

	else  
		# LOCAL

		# First send a stop to the process - can be local or remote
		# this should invoke a gentle quit from processes which inherit from 
		# yarp Module. Do it 3 times.

		killattempts=0
		noquitport=0

		while [ $killattempts -lt 3 ] && [ $noquitport -eq 0 ]
		do

			# check if we have a quit port

			yarp exists $name/quit
			stat=$?
			if [ $stat -ne 0 ]
			then
				noquitport=1
			else

				echo "Sending quit to $name/quit"
				# this is started in the bacground because sometimes it just 
				# doesn't return in that case we have to kill it
				echo "quit" | yarp rpc $name/quit  &

				sleep 1
				PID=`ps -ef | grep -e "yarp" | grep -e "rpc" | grep -e "quit" | awk '{print $2}'`
				if [ ${#PID} -ne 0 ] 
				then
					kill $PID
				fi
			fi

			killattempts=$(( $killattempts + 1 ))

		done

		# check if the process is still running - if so try harder to kill it

		EXECFP=`echo $CMD | awk '{print $1}' `
		EXEC=${EXECFP##*/}
		PARAM1=`echo $CMD | awk '{print $2}' `
		PARAM2=`echo $CMD | awk '{print $3}' `

		PID=`ps -ef | grep -e "$EXEC" | grep -e "$PARAM1" | grep -e "$PARAM2" | awk '{print $2}'`
		if [ ${#PID} -ne 0 ] 
		then
			echo "still running - check again in 1 second"
			sleep 1
			PID=`ps -ef | grep -e "$EXEC" | grep -e "$PARAM1" | grep -e "$PARAM2" | awk '{print $2}'`
		fi
		if [ ${#PID} -ne 0 ] 
		then
			echo "still running - trying to send -TERM signal to local process $EXEC"
			kill $PID
		fi

		PID=`ps -ef | grep -e "$EXEC" | grep -e "$PARAM1" | grep -e "$PARAM2" | awk '{print $2}'`
		if [ ${#PID} -ne 0 ] 
		then
			echo "still running - check again in 1 second"
			sleep 1
			PID=`ps -ef | grep -e "$EXEC" | grep -e "$PARAM1" | grep -e "$PARAM2" | awk '{print $2}'`
		fi

		# check again
		PID=`ps -ef | grep -e "$EXEC" | grep -e "$PARAM1" | grep -e "$PARAM2" | awk '{print $2}'`
		if [ ${#PID} -ne 0 ] 
		then
			echo "trying to force kill local process $EXEC"
			kill -9 $PID
		fi
	
	fi

}

# NOT USED NOW due to use of ssh 
# kept here so that non-yarp-run linux only processes can be killed ;)
function kill_process () {
	machine=$1
	executable=$2

	killattempts=0
	killlevel=""
	killed=0

	if [ "X$machine" != "Xlocal" ]
	then
		sshprefix="ssh icub@$machine "
	fi

	while [ $killed -eq 0 ] && [ $killattempts -lt 6 ]
	do
		if [ $killattempts -gt 3 ] 
		then
			killlevel="-9"
		fi

		PID=(`${sshprefix}ps -ef | grep $2 | grep -v grep | awk '{print $2}'`)
		if [ ${#PID[0]} -ne 0 ] 
		then
			echo "kill local process $2 (PID=${PID[*]}) on $machine (attempt $killattempts)"
			${sshprefix}kill $killlevel ${PID[*]}
		else
			killed=1
		fi

		killattempts=$(( $killattempts + 1 ))
		
	done
}


