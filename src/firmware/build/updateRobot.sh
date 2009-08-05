#!/usr/bin/env bash

filename=$1

cat $filename |
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

	#skip empty lines
	if [ "k$parLine" = "k" ]; then
		continue
	fi

	echo $cmd
	$cmd
	ret=$?
	#parse return values
	case $ret in
		0) echo "--> ALL OK" ;;
		255) echo "--> Error: INVALID_CMD_STRING" ;;
		254) echo "--> Error: INVALID_PARAM_TYPE" ;;
		253) echo "--> Error: INVALID_PARAM_NUM"	;;
		252) echo "--> Error: INVALID_PARAM_BOARD_ID" ;;
		251) echo "--> Error: INVALID_PARAM_FILE" ;;
		246) echo "--> Error: ERR_NO_BOARDS_FOUND" ;;
		245) echo "--> Error: ERR_BOARD_ID_NOT_FOUND" ;;
		244) echo "--> Error: ERR_UNKNOWN" ;;
		236) echo "--> Error: DOWNLOADERR_NOT_CONNECTED" ;;
		235) echo "--> Error: DOWNLOADERR_BOARD_NOT_SEL" ;;
		234) echo "--> Error: DOWNLOADERR_FILE_NOT_SEL"	;;
		233) echo "--> Error: DOWNLOADERR_FILE_NOT_OPEN" ;;
		232) echo "--> Error: DOWNLOADERR_BOARD_NOT_START" ;;
		231) echo "--> Error: DOWNLOADERR_TRANSFER_ERROR" ;;
		*) echo "Unknown return value" ;;
	esac
done
