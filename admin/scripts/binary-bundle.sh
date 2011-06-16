#!/bin/bash

BASE=`pwd`

DIRNAME=yarp-sim-win32-bin-`date +"%Y%m%d"`

#YARP_BIN=$YARP_DIR/binary-helper/bin
#SIM_BASE=$ICUB_DIR/src/iCubSimulation
#SIM_BIN=$SIM_BASE/binary-helper


YARP_BIN=$YARP_DIR/build/mingw/bin
SIM_BASE=$ICUB_DIR/src/iCubSimulation
SIM_BIN=$ICUB_DIR/build/mingw/bin

INV="$YARP_BIN/yarp.exe $YARP_BIN/yarpserver.exe $YARP_BIN/yarpview2.exe $YARP_BIN/yarpdev.exe $YARP_BIN/yarphear.exe  $SIM_BIN/iCub_YAIS.exe"
#rm $INV

#cd $YARP_DIR
#./scripts/compile-binaries.sh -DENABLE_mymod_wxsdl:BOOL=ON

#cd $ICUB_DIR/src/iCubSimulation
#$ICUB_DIR/scripts/compile-binaries.sh

cd $BASE

mkdir -p $DIRNAME
cd $DIRNAME
for f in $INV; do
    echo "Getting $f"
    cp $f .
done

echo "Canonicalizing yarpview2.exe name to yarpview.exe"
mv yarpview2.exe yarpview.exe

echo "Canonicalizing iCub_YAIS.exe name to iCubSimulation.exe"
mv iCub_YAIS.exe iCubSimulation.exe
editbin /STACK:30000000 iCubSimulation.exe  # fix stack size issue

for f in `ls *.exe`; do
    echo /usr/*mingw*/bin/strip $f
    /usr/*mingw*/bin/strip $f
done

mkdir -p conf
cp $SIM_BASE/conf/*.ini conf
for f in `ls conf/*.ini`; do
    echo "Configuration file $f"
    unix2dos $f
done
mkdir -p textures
cp $SIM_BASE/textures/*.ppm textures
cp $YARP_DIR/src/modules/fakebot/textures/*.ppm textures
for f in `ls textures/*.ppm`; do
    echo "Image $f"
done

(
cat<<EOF
This is a bundle of binaries from the YARP and iCub projects.

To use the iCub simulator, run:
  yarpserver.exe

Leave this running, and then start:
  iCubSimulation.exe

For source code and authorship information, see:
  http://yarp0.sourceforge.net
  http://www.robotcub.org/cvsweb/cvsweb.cgi/iCub/src/iCubSimulation/

Libraries used are ACE and ODE.
  ACE: http://www.cs.wustl.edu/~schmidt/ACE.html
  ODE: http://www.ode.org/

These binaries were built using the MinGW cross-compiler on Linux.

EOF
) > README.TXT

unix2dos README.TXT
echo "Added README.TXT"

cd ..
rm -f $DIRNAME.zip
zip -r $DIRNAME $DIRNAME
echo Check $DIRNAME.zip

echo "Suggest:"
echo "scp $DIRNAME.zip eris.liralab.it:/var/www/html/yarp/downloads/yarp/win32/"

