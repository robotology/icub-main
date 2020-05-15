#!/bin/bash

# run as ./admin/scripts/get-dependencies.sh
# run as superuser

. ./admin/scripts/config.sh

apt-get install libace-dev            # ACE
apt-get install libqwt-dev            # QWT for UZH
apt-get install libgsl0-dev           # math library
apt-get install libncurses5-dev       # on windows, conio.h -- see simio
apt-get install libcv-dev             # OpenCV, image processing
apt-get install libsdl1.2-dev         # SDL used by simulator
apt-get install libv4l-dev            # V4L
apt-get install libv4lconvert0        # V4L

# the following is not too important
apt-get install libboost-dev libboost-regex-dev  # Boost C++ library (for extend)
apt-get install libboost-program-options-dev     # More Boost (for extend)
apt-get install libboost-graph-dev               # For orocos KDL
