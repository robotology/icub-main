#!/bin/bash
# #############################################################################
#
# DESCRIPTION: this script contains all the cariable s needed to produce iCub
# binary packages
#
# AUTHOR : Matteo Brunettini <matteo.brunettini@iit.it>
#
# LATEST MODIFICATION DATE (YYYY-MM-DD): 2020-06-11
#
YCM_PACKAGE="ycm-cmake-modules"
YCM_PACKAGE_URL_bionic="https://launchpad.net/~robotology/+archive/ubuntu/ppa/+files/${YCM_PACKAGE}_0.11.1-1~ubuntu18.04~robotology1_all.deb"
YCM_PACKAGE_URL_buster="https://launchpad.net/~robotology/+archive/ubuntu/ppa/+files/${YCM_PACKAGE}_0.11.1-1_all.deb"
YCM_PACKAGE_URL_focal="https://launchpad.net/~robotology/+archive/ubuntu/ppa/+files/${YCM_PACKAGE}_0.11.1-1_all.deb"
SUPPORTED_DISTRO_LIST="buster bionic focal"
SUPPORTED_TARGET_LIST="amd64"
CMAKE_MIN_REQ_VER="3.12.0"
ICUB_DEPS_COMMON="libace-dev libc6 python3 libgsl0-dev libncurses5-dev libsdl1.2-dev subversion git gfortran libxmu-dev libode-dev wget unzip qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev libqt5svg5 libqt5opengl5-dev libopencv-dev freeglut3-dev libtinyxml-dev libblas-dev coinor-libipopt-dev liblapack-dev libmumps-dev qml-module-qtmultimedia qml-module-qtquick-dialogs qml-module-qtquick-controls libedit-dev libeigen3-dev libjpeg-dev libsimbody-dev libxml2-dev libjs-underscore portaudio19-dev ${YCM_PACKAGE}"
ICUB_DEPS_bionic="libode6"
ICUB_DEPS_focal="libode8"
ICUB_DEPS_buster="libode8"
ICUB_REPO_URL="https://github.com/robotology/icub-main"
ICUB_PACKAGE_MAINTAINER="Matteo Brunettini <matteo.brunettini@iit.it>"
export ICUB_DEBIAN_REVISION_NUMBER="1" # Always use a revision number >=1
