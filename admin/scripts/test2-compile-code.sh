#!/bin/bash

type=lenny

. ./admin/scripts/compile-config/$type/config.sh
. ./admin/scripts/compile-config/common.sh
. ./admin/scripts/compile-config/helpers.sh

SOURCE=$PWD
echo Working in directory $SOURCE | tee should_report.txt
./admin/scripts/test2-compilable.sh $type main || echo done compiling



