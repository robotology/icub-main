#!/bin/bash

type=lenny

SOURCE=$PWD
echo Working in directory $SOURCE | tee should_report.txt
./admin/scripts/test2-compilable.sh $type main || echo done compiling
./admin/scripts/test2-upload-report.sh $type main




