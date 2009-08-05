#!/bin/sh

for node in $@
  do
  if yarp exists /$node; then
	  echo "ok $node exists"
  else
	  echo "$node does NOT exist"
	  exit 0
  fi
done

exit 1
