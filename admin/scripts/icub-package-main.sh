#!/bin/bash

export SOURCE_TMP_DIR=./package-tmp
export MODULE=iCub
export REL=1.0.2
export TARFILE=$MODULE-src-$REL.tar.gz
export DEPFILE=$MODULE-dep-$REL.txt
export URL=https://robotcub.svn.sourceforge.net/svnroot/robotcub/trunk/iCub
./admin/scripts/icub-package-make-tar.sh
scp $TARFILE babybot@eris.liralab.it:/var/www/html/iCub/downloads/src/
rm $TARFILE

cp ./admin/scripts/current_dependencies.txt $DEPFILE
scp $DEPFILE babybot@eris.liralab.it:/var/www/html/iCub/downloads/src/

#export SOURCE_TMP_DIR=./package-tmp
##export MODULE=yarp2
#export TARFILE=yarp4iCub-src-$REL.tar.gz
#export URL=https://yarp0.svn.sourceforge.net/svnroot/yarp0/trunk/yarp2
#./admin/scripts/icub-package-make-tar.sh
#scp $TARFILE babybot@eris.liralab.it:/var/www/html/iCub/downloads/src/
#rm $TARFILE

echo "Done"
