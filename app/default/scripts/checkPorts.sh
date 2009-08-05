#!/bin/sh

for port in $@
  do
  if yarp exists $port; then
	  echo "ok $port exists"
  else
	  echo "$port does not exist"
	  exit 0
  fi
done

exit 1

