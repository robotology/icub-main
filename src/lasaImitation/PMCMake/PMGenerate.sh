#!/bin/bash
for fold in *
do
  if [ -d $fold ]
  then
    if [ -f $fold/CMakeLists.txt.pm ]
    then
      ./PMCMake/PMSubGenerate.sh $fold
    fi
  fi
done

