#!/bin/bash

base=`pwd`
fname="$3/$1"
tname="$2/$1"
org="$4"

tname=`echo $tname | sed "s|/\./|/|g"`
fname=`echo $fname | sed "s|/\./|/|g"`


if ( echo $fname | egrep -q "(/CVS)|(/modules)" ); then
  echo -n
else
    echo "echo \"*** Working on $fname\""
    if [ ! -e $tname ]; then
	if [ -d $fname ] ; then
	    echo "echo mkdir $tname"
	    echo "mkdir $tname"
	else
	    echo "echo cp $fname $tname"
	    echo "cp $fname $tname"
	fi    
	root=`echo $tname | sed "s|/[^/]*$||"`
	extra=`echo $tname | sed "s|.*/||"`
	echo "echo cd $root"
	echo "cd $root"
	cd $org
	type=`./check-type.sh $fname`
	echo "echo File type is [$type]"
	if [ "k$type" = "kbinary" ]; then
	    echo "echo cvs add -kb $extra"
	    echo "cvs add -kb $extra"
	else
	    echo "echo cvs add $extra"
	    echo "cvs add $extra"
	fi
	echo "echo cd $base"
	echo "cd $base"
    else
	if [ ! -d $fname ] ; then
	    cmp -s $fname $tname || (
		echo "echo cp $fname $tname"
		echo "cp $fname $tname"
	    )
	fi
    fi
fi



