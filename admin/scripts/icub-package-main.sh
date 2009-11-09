#!/bin/bash

SOURCE_TMP_DIR=./package-tmp
MODULE=iCub
REL=1.0.1
TARFILE=$MODULE-src-$REL.tar.gz
DEPFILE=$MODULE-dep-$REL.txt
URL=https://robotcub.svn.sourceforge.net/svnroot/robotcub/trunk/iCub
./admin/scripts/icub-package-make-tar.sh
scp $TARFILE babybot@eris.liralab.it:/var/www/html/iCub/downloads/src/
rm $TARFILE

SOURCE_TMP_DIR=./package-tmp
MODULE=yarp
TARFILE=$MODULE-src-$REL.tar.gz
DEPFILE=$MODULE-dep-$REL.txt
URL=https://yarp0.svn.sourceforge.net/svnroot/yarp0/trunk/yarp2
./admin/scripts/icub-package-make-tar.sh
scp $TARFILE babybot@eris.liralab.it:/var/www/html/iCub/downloads/src/
rm $TARFILE

cp ./admin/scripts/current_dependencies.txt $DEPFILE
scp $DEPFILE babybot@eris.liralab.it:/var/www/html/iCub/downloads/src/
rm $DEPFILE
