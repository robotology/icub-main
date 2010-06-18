#!/bin/bash

type=$1

EXPECTED_ARGS=1
 
if [ $# -ne $EXPECTED_ARGS ]
    SOURCE=$PWD
    echo Working in directory $SOURCE | tee should_report.txt
    ./admin/scripts/test2-compilable.sh $type main || echo done compiling
    ./admin/scripts/test2-upload-report.sh $type main
then
   echo "Usage: `basename $0` {distribution}"
fi





