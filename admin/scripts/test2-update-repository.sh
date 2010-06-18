#!/bin/bash

. ./admin/scripts/compile-config/common.sh
. ./admin/scripts/compile-config/helpers.sh

#run from $ICUB_ROOT as ./admin/scripts/update-repository.sh

std_timeout 600 svn update > svnlog.txt
cat svnlog.txt | grep -v "svn update" | egrep -v "^\? " | egrep -v "^M " > svnlog2.txt
cat svnlog2.txt | grep "At revision\|Updated to revision" > lastRevision.txt
