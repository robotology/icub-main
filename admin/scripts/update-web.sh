#!/bin/bash

# run as ./admin/scripts/update-web.sh

. ./admin/scripts/config.sh

if compile_dox; then
    cd doc && rsync --rsh="ssh -x -l $WEB_USER" --modify-window=2 -lavzP . $WEB_SERVER:$WEB_DOC_DIR
else
    cd $ICUB_DIR/doc/$WEB_DOC_SUFFIX && rsync --rsh="ssh -x -l $WEB_USER" --modify-window=2 -lavzP . $WEB_SERVER:$WEB_DOC_DIR/$WEB_DOC_SUFFIX
fi

#if is_msvc8; then
#    cd doc/report-msvc8 && rsync --rsh="ssh -x -l $WEB_USER" --modify-window=2 -lauvzP . $WEB_SERVER:$WEB_DOC_DIR/report-msvc8
#else
#    cd doc && rsync --rsh="ssh -x -l $WEB_USER" --modify-window=2 -lauvzP . $WEB_SERVER:$WEB_DOC_DIR
#fi


