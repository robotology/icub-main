#!/bin/bash

echo "Checkout code from $URL"
mkdir $SOURCE_TMP_DIR
cd $SOURCE_TMP_DIR

svn co $URL

echo "Clean svn files"
find ./ -iname .svn -exec rm -rf {} \;

versionFile=$MODULE/VERSION
echo "iCub snapshot version $REL" > versionFile
echo "Built on `date`" >> versionFile
echo "See $DEPFILE for list of library dependencies." >> versionFile

cp ./admin/scripts/current_dependencies.txt $MODULE/$DEPFILE

echo "Preparing tar file"
tar cvfz ../$TARFILE $MODULE

echo "Cleaning tmp dir"
cd ..
rm $SOURCE_TMP_DIR -rf

echo "Done"








