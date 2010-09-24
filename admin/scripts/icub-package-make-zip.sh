#!/bin/bash

echo "Checkout code from $URL"
mkdir $SOURCE_TMP_DIR
cd $SOURCE_TMP_DIR

svn co $URL

echo "Clean svn files"
echo "Clean svn files"
find ./ -type d -iname .svn -exec rm -rf {} \;
echo "Fix eol for dos"
find ./ -type f -name '*.txt' -o -name *.ini -o -name *.cpp -o -name *.c -o -name *.cc -o -name *.cmake -o -name *.cmake -exec unix2dos {} \;

versionFile=$MODULE/VERSION
echo "iCub snapshot version $REL" > $versionFile
echo "Built on `date`" >> $versionFile
echo "See $DEPFILE for list of library dependencies." >> $versionFile

cp ../admin/scripts/current_dependencies.txt $MODULE/$DEPFILE

echo "Preparing zip file"
zip -r ../$ARCHFILE $MODULE

echo "Cleaning tmp dir"
cd ..
rm $SOURCE_TMP_DIR -rf

echo "Done"








