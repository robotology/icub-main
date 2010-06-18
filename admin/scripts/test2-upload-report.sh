#!/bin/bash

type=$1

. ./admin/scripts/compile-config/$type/config.sh
. ./admin/scripts/compile-config/common.sh
. ./admin/scripts/compile-config/helpers.sh

cd $REPORT_DIR && rsync --rsh="ssh -x -l $WEB_USER" --modify-window=2 -lavzP . $WEB_SERVER:$WEB_DOC_DIR/$2/reports/$type






