#!/bin/bash

type=$1

. ./admin/scripts/compile-config/$type/config.sh
. ./admin/scripts/compile-config/common.sh
. ./admin/scripts/compile-config/helpers.sh

cd $ICUB_DIR/doc/report && rsync --rsh="ssh -x -l $WEB_USER" --modify-window=2 -lavzP . $WEB_SERVER:$WEB_DOC_DIR/reports/$WEB_DOC_SUFFIX/$type





