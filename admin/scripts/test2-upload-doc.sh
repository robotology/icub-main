#!/bin/bash

# run as ./admin/scripts/test2-upload-doc.sh

. ./admin/scripts/compile-config/common.sh
. ./admin/scripts/compile-config/doxygen/config.sh

doc=$ICUB_DIR/doc

cd $doc && rsync --rsh="ssh -x -l $WEB_USER" --modify-window=2 -lavzP . $WEB_SERVER:$WEB_DOC_DIR/$WEB_DOC_SUFFIX


