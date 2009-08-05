#!/bin/bash

fname="$1"
tname="$2/$1"

tname=`echo $tname | sed "s|/\./|/|g"`
base=`pwd`

if ( echo $fname | egrep -q "(/CVS)" ); then
  echo -n
else
    echo "# Removal check for $fname ($tname)"
    if [ ! -e $tname ]; then
	removable=true
	if [ -d $fname ] ; then
	    echo "# ignoring old directory $fname"
	    removable=false  #CVS ONLY
	else
	    echo "rm $fname"
	fi    
	root=`echo $fname | sed "s|/[^/]*$||"`
	extra=`echo $fname | sed "s|.*/||"`
	if $removable; then
	    echo "cd $root"
	    echo "cvs remove $extra"
	    echo "cd $base"
	fi
    fi
fi



