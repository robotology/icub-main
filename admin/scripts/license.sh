#!/bin/bash

# Copyright: (C) 2011 RobotCub Consortium
# Author: Lorenzo Natale 
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

# Inspired from similar script in yarp by Paul Fitzpatrick

echo "Make sure you run this as ./admin/scripts/update-license"

# reset log
echo -n | tee licenses-all.txt

### put back when done
#rm -rf license_check
#svn export . license_check

rm licenses-all.txt
rm licenses-authors.txt
rm licenses-bad.txt
rm licenses-good.txt 

prefix_dir="./license_check/main/"
prefix_dir="./main/src/"

#`cd $prefix svn up`
file_list=`cd $prefix_dir; find . -type f -iname "*.cpp" -or -iname "*.c" -or -iname "*.h" -or -iname "*.hpp" -or -iname "CMakeLists.txt" -or -iname "*.cmake"`

for f in $file_list
do
    ./admin/scripts/license-collect.pl $prefix_dir/$f >> licenses-all.txt
done

./admin/scripts/license-aggregate.pl licenses-all.txt

rm -rf license_check

