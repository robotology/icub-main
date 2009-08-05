#!/bin/bash

arm=$2

#if arm=empty, read from file
if let "${#arm}==0"
then 
exec 6<&0
exec < arm.txt
read row
arm=${row}
exec 0<&6 6<&-
fi 

#if arm still empty, default="right"
if let "${#arm}==0"
then arm="right" 
fi 

#if arm not in {"right", "left"}, default="right"
if [ "${arm}" != "right" -a "${arm}" != "left" ]
then arm="right"
fi

#save into file
echo ${arm} > arm.txt

case "$1" in
  'start')
  ./scripts/1_run.sh ${arm}
  ;;
  'stop')
  ./scripts/4_stop.sh
  ;;
  'connect')
  ./scripts/2_connect.sh ${arm}
  ;;
  'disconnect')
  ./scripts/3_disconnect.sh ${arm}
  ;;
esac
exit 0
