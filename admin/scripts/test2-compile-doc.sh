#!/bin/bash

SOURCE=$PWD

EXPECTED_ARGS=1
 
if [ $# -ne $EXPECTED_ARGS ];
then
   echo "Usage: `basename $0` {main/contrib}"
else
    base=$PWD
    cd $ICUB_ROOT/$1
    doxygen ./conf/Doxyfile.txt
    cd $base
fi




