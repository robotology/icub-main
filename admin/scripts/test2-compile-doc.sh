#!/bin/bash

SOURCE=$PWD

EXPECTED_ARGS=1
 
if [ $# -ne $EXPECTED_ARGS ];
then
    base=$PWD
    cd $ICUB_ROOT/$1
    doxygen ./conf/Doxyfile.txt
    cd $base
else
   echo "Usage: `basename $0` {main/contrib}"
fi



