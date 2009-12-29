#!/bin/bash
if [ $# -ne 1 ]
then
  echo "Usage: `basename $0` SubProjectName"
  exit
fi

echo Setting up sub-project: $1
sed -e "s/MYLIBNAME/$1/g" < ./PMCMake/Config.cmake.template > ./$1/$1Config.cmake
cat ./$1/CMakeLists.txt.pm ./PMCMake/CMakeLists.txt.template > ./$1/CMakeLists.txt

#cp CMakeLists.txt.pm CMakeLists.txt
#TAG=PM_PROJECT_NAME
#VAL=`grep $TAG PMakeLists.txt | sed -e "s/$TAG=\([A-Za-z0-9_]\)/\1/"`
#sed -e "s/$TAG/$VAL/g" CMakeLists.txt.pm
