#!/bin/bash

echo "Checkout code from $URL"
mkdir $SOURCE_TMP_DIR
cd $SOURCE_TMP_DIR

svn co $URL

echo "Clean svn files"
find ./ -iname .svn -exec rm -rf {} \;

echo "Preparing tar file"
tar cvfz ../$TARFILE $MODULE

echo "Cleaning tmp dir"
cd ..
rm $SOURCE_TMP_DIR -rf

echo "Done"








