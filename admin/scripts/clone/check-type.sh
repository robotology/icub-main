#!/bin/bash

tname="$1"

key=`echo $tname | sed "s|[^a-zA-Z0-9\.]|_|g"`

mkdir -p types
if [ ! -e types/$key ]; then
  root=`echo $tname | sed "s|/[^/]*$||"`
  extra=`echo $tname | sed "s|.*/||"`

  #echo "Checking $root $extra"

  org=`pwd`

  cd $root
  type=`cvs log $extra | grep "keyword substitution:" | sed "s/.*: //"`
  cd $org
  echo $type > types/$key
  #echo "Type is $type"
else
  type=`cat types/$key`
fi

if [ "k$type" = "kb" ]; then
    echo "binary"
else
    echo "text"
fi


