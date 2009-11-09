#!/bin/bash

src=`cat source.txt`
dest=`cat target.txt`

orgdir=`pwd`
srcdir="$orgdir/state/$src"
destdir="$orgdir/state/$dest"

(
cd $srcdir
find . -exec $orgdir/prepare-action.sh {} $destdir $srcdir $orgdir \;

cd $destdir
find . -depth -exec $orgdir/prepare-removal.sh {} $srcdir $destdir \;
) | tee apply.sh

chmod u+x apply.sh
