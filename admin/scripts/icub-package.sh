#!/bin/bash

# Copyright: RobotCub Consortium
# Author: Lorenzo Natale

export SOURCE_TMP_DIR=./package-tmp
export MODULE=iCub
export REL=1.1.1
export ARCHFILE_LINUX=$MODULE-src-$REL.tar.gz
export ARCHFILE_WINDOWS=$MODULE-src-$REL.zip
export DEPFILE=$MODULE-dep-$REL.txt
export URL=https://robotcub.svn.sourceforge.net/svnroot/robotcub/trunk/$MODULE
export TAG_URL=https://robotcub.svn.sourceforge.net/svnroot/robotcub/tags/$MODULE$REL

mkdir $SOURCE_TMP_DIR
cd $SOURCE_TMP_DIR

versionFile=VERSION
echo "iCub snapshot version $REL" > $versionFile
echo "Built on `date`" >> $versionFile
echo "See $DEPFILE for list of library dependencies." >> $versionFile

########### Linux
echo "Checkout code from $TAG_URL"

svn export $TAG_URL $MODULE

#store this file for later upload
cp $MODULE/admin/scripts/current_dependencies.txt $DEPFILE

#preparing repository
cp $MODULE/admin/scripts/current_dependencies.txt $MODULE/$DEPFILE
cp $versionFile $MODULE

echo "Preparing tar file"
tar cvfz $ARCHFILE_LINUX $MODULE

echo "Cleaning checkout"
rm $MODULE -rf

echo "Done Linux"

########### Windows
echo "Checkout code from $URL"

svn export $TAG_URL $MODULE --native-eol CRLF

#store this file for later upload
cp $MODULE/admin/scripts/current_dependencies.txt $DEPFILE

#preparing repository
cp $MODULE/admin/scripts/current_dependencies.txt $MODULE/$DEPFILE
cp $versionFile $MODULE

echo "Preparing zip file"
zip -r $ARCHFILE_WINDOWS $MODULE

echo "Cleaning checkout"
rm $MODULE -rf

echo "Done Windows"

################ Upload

scp $ARCHFILE_LINUX babybot@eris.liralab.it:/var/www/html/iCub/downloads/src/

scp $ARCHFILE_WINDOWS babybot@eris.liralab.it:/var/www/html/iCub/downloads/src/

scp $DEPFILE babybot@eris.liralab.it:/var/www/html/iCub/downloads/src/

#scp $ARCHFILE_WINDOWS natta,robotcub@frs.sourceforge.net:/home/frs/project/r/ro/robotcub/snapshots/win/$ARCHFILE_WINDOWS
#scp $ARCHFILE_LINUX natta,robotcub@frs.sourceforge.net:/home/frs/project/r/ro/robotcub/snapshots/linux/$ARCHFILE_LINUX

echo "Done"

cd ..
#rm $SOURCE_TMP_DIR -rf


