#!/bin/bash

# Copyright: 2010 RobotCub Consortium
# Author: Lorenzo Natale


export SOURCE_TMP_DIR=./package-tmp
export REL=$1
export MODULE=main
export ARCHFILE_LINUX=iCub-$MODULE-src-$REL.tar.gz
export ARCHFILE_WINDOWS=iCub-$MODULE-src-$REL.zip
export DEPFILE=iCub-$MODULE-dep-$REL.txt

if [ "k$1" =  "k" ]; then
 export SVN_URL="https://github.com/robotology/icub-main/trunk"
else
 export SVN_URL="https://github.com/robotology/icub-main/tags/v$REL"
fi

mkdir $SOURCE_TMP_DIR
cd $SOURCE_TMP_DIR

versionFile=VERSION
echo "iCub snapshot version $REL" > $versionFile
echo "Built on `date`" >> $versionFile
echo "See $DEPFILE for list of library dependencies." >> $versionFile

########### Linux
echo "Checkout code from $SVN_URL"

svn export $SVN_URL icub-main/

#store this file for later upload
cp iCub/admin/scripts/current_dependencies.txt $DEPFILE

#preparing repository
cp iCub/admin/scripts/current_dependencies.txt icub-main/$DEPFILE
cp $versionFile icub-main/

echo "Preparing tar file"
tar cvfz $ARCHFILE_LINUX icub-main

echo "Cleaning checkout"
rm -rf icub-main

echo "Done Linux"

########### Windows
echo "Checkout code from $SVN_URL"

svn export $SVN_URL icub-main --native-eol CRLF

#store this file for later upload
cp iCub/admin/scripts/current_dependencies.txt $DEPFILE

#preparing repository
cp iCub/admin/scripts/current_dependencies.txt icub-main/$DEPFILE
cp $versionFile icub-main/

echo "Preparing zip file"
zip -r $ARCHFILE_WINDOWS icub-main

echo "Cleaning checkout"
rm -rf icub-main

echo "Done Windows"

#REMOTE_DIR=iCub-$MODULE-$REL
#mkdir -p $REMOTE_DIR
#mv $ARCHFILE_LINUX $REMOTE_DIR
#mv $ARCHFILE_WINDOWS $REMOTE_DIR
#
#echo "rsync --recursive $REMOTE_DIR natta,robotcub@frs.sourceforge.net:/home/frs/project/r/ro/robotcub/iCub-main"
#
################ old upload

#scp $ARCHFILE_LINUX babybot@wiki.icub.org:/var/www/html/iCub/downloads/src/
#scp $ARCHFILE_WINDOWS babybot@wiki.icub.org:/var/www/html/iCub/downloads/src/
#scp $DEPFILE babybot@wiki.icub.org:/var/www/html/iCub/downloads/src/

#scp $ARCHFILE_WINDOWS natta,robotcub@frs.sourceforge.net:/home/frs/project/r/ro/robotcub/snapshots/win/$ARCHFILE_WINDOWS
#scp $ARCHFILE_LINUX natta,robotcub@frs.sourceforge.net:/home/frs/project/r/ro/robotcub/snapshots/linux/$ARCHFILE_LINUX

echo "Done"

cd ..
#rm $SOURCE_TMP_DIR -rf
