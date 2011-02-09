#!/bin/bash

# Copyright: (C) 2011 RobotCub Consortium
# Author: Lorenzo Natale 
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

# Inspired from similar script in yarp by Paul Fitzpatrick

echo "Make sure you run this as ./admin/scripts/update-license"

# reset change log
echo -n | tee license-changes.txt
echo -n | tee license-good.txt
echo -n | tee license-odd.txt

### put back when done
#rm -rf license_check
#svn export . license_check

prefix_dir="license_check/main/src/core"

file_list=`cd $prefix_dir; find . -type f -iname "*.cpp" -or -iname "*.c" -or -iname "*.h" -or -iname "*.txt" -or -iname "*.cmake"`

rm licenses-all.txt
for f in $file_list
do
    ./admin/scripts/update-license-single.pl $prefix_dir/$f >> licenses-all.txt
done

./admin/scripts/aggregate-license.pl licenses-all.txt
