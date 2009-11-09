#!/bin/bash

# run as ./admin/scripts/get-dependencies.sh
# run as superuser

. ./admin/scripts/config.sh

apt-get install libace-dev libcv-dev  # ACE and OpenCV
apt-get install qt3-dev-tools         # Qt build tools (qmake, moc, ui)
apt-get install libqt3-mt-dev         # Qt (UZH guis)
apt-get install libqwt-dev            # QWT for UZH
apt-get install libgsl0-dev           # math library
apt-get install libncurses5-dev       # on windows, conio.h -- see simio
apt-get install libcv-dev             # OpenCV, image processing
apt-get install libgtk2.0-dev libgtkmm-2.4-dev libglademm-2.4-dev  # GTK/MM GUIs
apt-get install libsdl1.2-dev         # SDL used by simulator

# the following is not too important
apt-get install libboost-dev libboost-regex-dev  # Boost C++ library (for extend)
apt-get install libboost-program-options-dev     # More Boost (for extend)
apt-get install libboost-graph-dev               # For orocos KDL
