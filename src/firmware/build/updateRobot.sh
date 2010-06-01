#!/usr/bin/env bash

filename=$1
count=0
arrayOfRet=0
arrayOfCmd=0

while read parLine; do
	canDevice=`echo $parLine | awk 'BEGIN { FS=" "}; {print $1}'`
	deviceNum=`echo "$parLine"|awk 'BEGIN { FS=" "}; {print $2}'`
	boardId=`echo "$parLine"|awk 'BEGIN { FS=" "}; {print $3}'`
	firmware=`echo "$parLine"|awk 'BEGIN { FS=" "}; {print $4}'`
	
	cmd="canLoader20 --canDeviceType $canDevice --canDeviceNum $deviceNum --boardId $boardId --firmware $firmware"

	# skip comments
	if [ "k$canDevice" = "k//" ]; then
		continue
	fi

	# skip empty lines
	if [ "k$parLine" = "k" ]; then
		continue
	fi

	echo $cmd
	$cmd
	ret=$?

	count=`expr $count + 1`
	# echo "Iteration number $count"

	arrayOfRet[$count]=$ret
	arrayOfCmd[$count]=`echo $parLine`
	# echo "Current ret[$count] is: ${arrayOfRet[$count]}"
done < $filename

# echo "Length of array is $count"
i=1
while [ $i -lt `expr $count + 1` ]; do
	#parse return values
	case ${arrayOfRet[$i]} in
		0) echo "${arrayOfCmd[$i]} --> ALL OK" ;;
		255) echo "${arrayOfCmd[$i]} --> Error: INVALID_CMD_STRING" ;;
		254) echo "${arrayOfCmd[$i]} --> Error: INVALID_PARAM_TYPE" ;;
		253) echo "${arrayOfCmd[$i]} --> Error: INVALID_PARAM_NUM"	;;
		252) echo "${arrayOfCmd[$i]} --> Error: INVALID_PARAM_BOARD_ID" ;;
		251) echo "${arrayOfCmd[$i]} --> Error: INVALID_PARAM_FILE" ;;
		246) echo "${arrayOfCmd[$i]} --> Error: ERR_NO_BOARDS_FOUND" ;;
		245) echo "${arrayOfCmd[$i]} --> Error: ERR_BOARD_ID_NOT_FOUND" ;;
		244) echo "${arrayOfCmd[$i]} --> Error: ERR_UNKNOWN" ;;
		236) echo "${arrayOfCmd[$i]} --> Error: DOWNLOADERR_NOT_CONNECTED" ;;
		235) echo "${arrayOfCmd[$i]} --> Error: DOWNLOADERR_BOARD_NOT_SEL" ;;
		234) echo "${arrayOfCmd[$i]} --> Error: DOWNLOADERR_FILE_NOT_SEL"	;;
		233) echo "${arrayOfCmd[$i]} --> Error: DOWNLOADERR_FILE_NOT_OPEN" ;;
		232) echo "${arrayOfCmd[$i]} --> Error: DOWNLOADERR_BOARD_NOT_START" ;;
		231) echo "${arrayOfCmd[$i]} --> Error: DOWNLOADERR_TRANSFER_ERROR" ;;
		*) echo "${arrayOfCmd[$i]} --> Unknown return value" ;;
	esac
        let i++
done
