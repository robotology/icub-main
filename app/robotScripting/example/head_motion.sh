#!/bin/bash

count=1
while [ "$count" -le "$1" ]
do
  azi=$RANDOM
  ele=$RANDOM
  ver=$RANDOM

  let "azi %= 60"
  let "ele %= 10"
  let "azi -= 30"
  let "ele -= 5"
  let "ver %= 15"

  let "count += 1"
  
  echo "abs "$azi" "$ele" "$ver"" | yarp write ... /iKinGazeCtrl/angles:i
  sleep "$2"
done

echo "abs 0.0 0.0 1.0" | yarp write ... /iKinGazeCtrl/angles:i




