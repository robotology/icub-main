#!/bin/bash
# #############################################################################
#
# DESCRIPTION: this script contains all the cariable s needed to produce iCub
# binary packages
#
# AUTHOR : Matteo Brunettini <matteo.brunettini@iit.it>
#          Nicolo' Genesio <nicolo.genesio@iit.it>
#
# LATEST MODIFICATION DATE (YYYY-MM-DD): 2023-02-24
#
YCM_PACKAGE="ycm-cmake-modules"
YCM_REQUIRED_VERSION="0.15.1"
YCM_PACKAGE_URL="https://github.com/robotology/ycm/releases/download/v${YCM_REQUIRED_VERSION}/${YCM_PACKAGE}-${YCM_REQUIRED_VERSION}-all.deb"

YARP_REQUIRED_VERSION="3.8.0"
YARP_PACKAGE_URL_buster="https://github.com/robotology/yarp/releases/download/v${YARP_REQUIRED_VERSION}/yarp-${YARP_REQUIRED_VERSION}-2.buster_amd64.deb"
YARP_PACKAGE_URL_focal="https://github.com/robotology/yarp/releases/download/v${YARP_REQUIRED_VERSION}/yarp-${YARP_REQUIRED_VERSION}-2.focal_amd64.deb"
YARP_PACKAGE_URL_jammy="https://github.com/robotology/yarp/releases/download/v${YARP_REQUIRED_VERSION}/yarp-${YARP_REQUIRED_VERSION}-2.jammy_amd64.deb"

APT_OPTIONS="-q -y"

SUPPORTED_DISTRO_LIST="buster focal jammy"
SUPPORTED_TARGET_LIST="amd64"
CMAKE_MIN_REQ_VER="3.16.0"
ICUB_DEPS_COMMON="libode8 libace-dev libboost-filesystem-dev libboost-system-dev libboost-thread-dev libc6 python3 libgsl0-dev libncurses5-dev libsdl1.2-dev subversion git gfortran libxmu-dev libode-dev wget unzip qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev libqt5svg5 libqt5opengl5-dev libopencv-dev freeglut3-dev libtinyxml-dev libblas-dev coinor-libipopt-dev liblapack-dev libmumps-dev qml-module-qtmultimedia qml-module-qtquick-dialogs qml-module-qtquick-controls qml-module-qt-labs-settings libqcustomplot-dev qml-module-qt-labs-folderlistmodel libedit-dev libeigen3-dev libjpeg-dev libsimbody-dev libxml2-dev libjs-underscore portaudio19-dev libgtkmm-2.4-dev libi2c-dev libv4l-dev ${YCM_PACKAGE}"
ICUB_DEPS_jammy=""
ICUB_DEPS_focal=""
ICUB_DEPS_buster=""
ICUB_PACKAGE_MAINTAINER="Matteo Brunettini <matteo.brunettini@iit.it>"
export ICUB_DEBIAN_REVISION_NUMBER="1" # Always use a revision number >=1

ICUB_CMAKE_OPTIONS="\
 -DCMAKE_BUILD_TYPE=Release \
 -DICUB_USE_SDL=ON \
 -DIPOPT_DIR=/usr \
 -DICUB_USE_IPOPT=ON \
 -DICUB_USE_GLUT=ON \
 -DENABLE_icubmod_canmotioncontrol=OFF \
 -DENABLE_icubmod_cartesiancontrollerclient=ON \
 -DENABLE_icubmod_cartesiancontrollerserver=ON \
 -DENABLE_icubmod_fakecan=ON \
 -DENABLE_icubmod_gazecontrollerclient=ON \
 -DENABLE_icubmod_skinprototype=ON \
 -DENABLE_icubmod_socketcan=ON \
 -DENABLE_icubmod_static_grabber=ON \
 -DENABLE_icubmod_xsensmtx=OFF \
 -DYARP_FORCE_DYNAMIC_PLUGINS=ON"
CMAKE_OPTIONS_focal=""
CMAKE_OPTIONS_jammy=""
CMAKE_OPTIONS_buster=""
