#!/bin/bash

# run as ./admin/scripts/autocheck.sh

. ./admin/scripts/config.sh

export ICUB_ROOT=$PWD

rm -f should_report.txt

(
# all this needs the "timeout" and "realpath" command

std_timeout 600 svn update > svnlog.txt
cat svnlog.txt | grep -v "svn update" | egrep -v "^\? " | egrep -v "^M " > svnlog2.txt
cat svnlog2.txt | grep "At revision\|Updated to revision" > lastRevision.txt

SOURCE=$PWD
echo Working in directory $SOURCE | tee should_report.txt
./admin/scripts/compilable.sh || echo done compiling
chmod u+x ./admin/scripts/update-web.sh # sad CVS accident

if compile_dox; then
    ./admin/scripts/update-doc.sh || echo done documenting
fi

rm -rf ./doc/$WEB_DOC_SUFFIX
mv ./doc/report ./doc/$WEB_DOC_SUFFIX

std_timeout 600 ./admin/scripts/update-web.sh
) | tee report.txt

