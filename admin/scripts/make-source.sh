#!/bin/bash

# Copyright: RobotCub Consortium
# Author: Lorenzo Natale

export SOURCE_TMP_DIR=./package-tmp
export MODULE=iCub
export REL=1.1.0
export ARCHFILE_LINUX=$MODULE-src-$REL.tar.gz
export ARCHFILE_WINDOWS=$MODULE-src-$REL.zip
export DEPFILE=$MODULE-dep-$REL.txt
export URL=https://robotcub.svn.sourceforge.net/svnroot/robotcub/trunk/iCub

mkdir $SOURCE_TMP_DIR
cd $SOURCE_TMP_DIR

versionFile=$SOURCE_TMP_DIR/$MODULE/VERSION
echo "iCub snapshot version $REL" > $versionFile
echo "Built on `date`" >> $versionFile
echo "See $DEPFILE for list of library dependencies." >> $versionFile

########### Linux
echo "Checkout code from $URL"

svn export $URL

cp $MODULE/admin/scripts/current_dependencies.txt $MODULE/$DEPFILE
cp $MODULE/admin/scripts/current_dependencies.txt $DEPFILE

echo "Preparing tar file"
tar cvfz $ARCHFILE_LINUX $MODULE

echo "Cleaning checkout"
cd ..
rm $MODULE -rf

echo "Done Linux"

########### Windows
echo "Checkout code from $URL"

svn export $URL --native-eol CRLF

cp $MODULE/admin/scripts/current_dependencies.txt $MODULE/$DEPFILE
cp $MODULE/admin/scripts/current_dependencies.txt $DEPFILE

echo "Preparing zip file"
zip -r $ARCHFILE_WINDOWS $MODULE

echo "Cleaning checkout"
cd ..
rm $MODULE -rf

echo "Done Windows"

################ Upload

scp $ARCHFILE_LINUX babybot@eris.liralab.it:/var/www/html/iCub/downloads/src/
rm $ARCHFILE_LINUX

scp $ARCHFILE_WINDOWS babybot@eris.liralab.it:/var/www/html/iCub/downloads/src/
rm $ARCHFILE_WINDOWS

scp $DEPFILE babybot@eris.liralab.it:/var/www/html/iCub/downloads/src/

echo "Done"

cd ..
rm $SOURCE_TMP_DIR -rf


