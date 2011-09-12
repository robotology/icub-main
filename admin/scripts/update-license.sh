#!/bin/bash

# Copyright: (C) 2011 RobotCub Consortium
# Author: Lorenzo Natale 
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

# Inspired from similar script in yarp by Paul Fitzpatrick

echo "Make sure you run this as ./admin/scripts/update-license.sh"

# reset log
echo -n | tee licenses-all.txt

### put back when done
#rm -rf license_check
#svn export . license_check

rm licenses-*.txt

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

(
echo "The iCub main package is distributed under the GPL 2.0 or later."
echo "A copy of the license is available at: http://www.robotcub.org/icub/license/gpl.txt"
echo
echo "The list of authors who contributed code to iCub main is:"
cat licenses-authors-compact.txt
echo
echo "The list of copyright holders for iCub main is:"
cat licenses-copyright-compact.txt
echo
cat<<EOF
The identifier "RobotCub Consortium" used in some copyright statements is 
equivalent to the following list of institutions:
  * Ecole Polytechnique Federale de Lausanne - Biologically-Inspired
    Robotics Group (BIRG) and Learning Algorithms and Systems Lab (LASA),
    Switzerland
  * IST Lisbon - Computer Vision and Robotics Lab Lisbon - Portugal
  * Italian Institute of Technology - Dept. of Robotics, Brain and
    Cognitive Sciences - Genova, Italy
  * Sant'Anna School of Advanced Studies - ARTS Lab - Pisa - Italy
  * Telerobot S.r.l. - Genova - Italy
  * University of Ferrara - Department of Biomedical Science - Human
    Physiology - Ferrara - Italy
  * University of Genova - LIRA-Lab, Dipartimento di Informatica,
    Sistemistica e Telematica - Genova - Italy
  * University of Hertfordshire - School of Computer Science - United
    Kingdom  * University of Uppsala - Department of Psychology -
    Uppsala - Sweden
  * University of Zurich - Artificial Intelligence Lab, Department of
    Information Technology - Zurich - Switzerland
  * [2005-2008] University of Salford - Centre for Robotics and  Automation
    Salford - United Kingdom
  * [2009-2010] The University of Sheffield, Dept. of Automatic Control &
    Systems Engineering, Sheffield, UK

EOF

) > main/COPYING_new

(
 echo "This is the list of authors who contributed code to the iCub main package:"
 echo
 cat licenses-authors-compact.txt
 echo

) > main/AUTHORS_new


