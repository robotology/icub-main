#!/bin/bash

if [ $# -ne 1 ] 
then
	echo "usage: $0 <ModuleNamePrefix>"
	echo " should use capitalized form : ModuleNamePrefix"
	exit -1
fi

prefix=$1
modulename=${prefix}Module

if [ "XX$prefix" == "XXTemplate" ]
then
	echo "Cannot create module starting with Template"
	exit -1
fi

echo "Creating new module $modulename"

# desc is Prefix with spaces inserted "ThisPrefix"-->"This Prefix"
desc=`echo $prefix | sed 's/\([a-z]\)\([A-Z]\)/\1 \2/g'`

# LCpref is prefix with "_" inserted and lowercase "ThisPrefix"-->"this_prefix"
LCpref=`echo $prefix | sed 's/\([a-z]\)\([A-Z]\)/\1_\2/g' | sed 's/\([A-Z]\)/\l\1/g' `
DIR=$LCpref

# UCpref is prefix with "_" inserted and uppercase "ThisPrefix"-->"THIS_PREFIX"
UCpref=`echo $prefix | sed 's/\([a-z]\)\([A-Z]\)/\1_\2/g' | sed 's/\([a-z]\)/\u\1/g' `


echo "Module Description: $desc"

# build new directories
echo "Creating Directory $DIR"
if [ -d "$DIR" ]
then
	echo "Module $prefix exists"
	exit -1
fi
SRC=$DIR/src
INC=$DIR/include/iCub/iha
CONF=$DIR/conf
mkdir -p $SRC
mkdir -p $INC
mkdir -p $CONF

TDIR=_template
TSRC=$TDIR/src
TINC=$TDIR/include/iCub/iha
TCONF=$TDIR/conf


# create the files from the template replacing the Template bits
sed "s/Template/$prefix/g" $TDIR/CMakeLists.txt > $DIR/CMakeLists.txt

sed "s/Template/$prefix/g" $TSRC/main.cpp |sed "s/DESC/$desc/g" | sed "s/TEMPLATE/$UCpref/g" | sed "s/template/$LCpref/g" |sed "s/ASTERIX/\*/g" > $SRC/main.cpp

sed "s/Template/$prefix/g" $TSRC/TemplateModule.cpp |sed "s/DESC/$desc/g" | sed "s/TEMPLATE/$UCpref/g" | sed "s/template/$LCpref/g" |sed "s/ASTERIX/\*/g"  > $SRC/$modulename.cpp

sed "s/Template/$prefix/g" $TINC/TemplateModule.h |sed "s/DESC/$desc/g" | sed "s/TEMPLATE/$UCpref/g" | sed "s/template/$LCpref/g" |sed "s/ASTERIX/\*/g" > $INC/$modulename.h

sed "s/Template/$prefix/g" $TCONF/ihaTemplate.ini |sed "s/DESC/$desc/g" | sed "s/TEMPLATE/$UCpref/g" | sed "s/template/$LCpref/g" |sed "s/ASTERIX/\*/g" > $CONF/iha$modulename.ini
