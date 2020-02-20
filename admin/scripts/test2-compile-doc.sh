#!/bin/bash

SOURCE=$PWD

EXPECTED_ARGS=1

DESTINATIONXML=generated-from-xml

if [ $# -ne $EXPECTED_ARGS ] && [ $# -ne $[$EXPECTED_ARGS+1] ];
then
   echo "Usage: `basename $0` {main/contrib}"
   echo "Optionally: add --xml to parse xmlfiles "
   exit
fi

#cleanup old
rm $ICUB_ROOT/$1/doc -rf
rm $ICUB_ROOT/$1/$DESTINATIONXML -rf

if [ -n $YARP_ROOT ]
 then
 # producing doxygen from xml
 if [ "$2" = "--xml" ];
 then
  echo "Preparsing xml"
  cd $ICUB_ROOT/$1
  mkdir $DESTINATIONXML
  list=`find . -iname *.xml | xargs`
  for i in $list
  do
    path=`dirname $i`
    filename=`basename $i`
    doxyfile=${filename%%.*}
    xsltproc --output $DESTINATIONXML/$doxyfile.dox $YARP_ROOT/scripts/yarp-module.xsl $i
  done
 fi
else
 echo "Parsing doxygen only"
fi

base=$PWD
cd $ICUB_ROOT/$1
doxygen ./conf/Doxyfile.txt
cd $base






