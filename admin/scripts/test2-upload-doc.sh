#!/bin/bash

# run as ./admin/scripts/test2-upload-doc.sh

. ./admin/scripts/doc-config/common.sh

EXPECTED_ARGS=1

if [ $# -ne $EXPECTED_ARGS ];
then
    echo "Usage: `basename $0` {main/contrib}"
else
    mod=$1
    doc=$ICUB_ROOT/$mod/doc

    cd $doc && rsync --rsh="ssh -x -l $WEB_USER" --modify-window=2 -lavzP --delete . $WEB_SERVER:$WEB_DOC_DIR/$mod/dox
fi



