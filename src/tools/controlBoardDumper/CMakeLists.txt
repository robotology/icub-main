# 
# Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
# Author: Francesco Nori
# email:  francesco.nori@iit.it
# website: www.robotcub.org
# Permission is granted to copy, distribute, and/or modify this program
# under the terms of the GNU General Public License, version 2 or any
# later version published by the Free Software Foundation.
#
# A copy of the license can be found at
# http://www.robotcub.org/icub/license/gpl.txt
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details
#


PROJECT(controlBoardDumper)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_EXECUTABLE(controlBoardDumper ${folder_source} ${folder_header})
target_link_libraries(controlBoardDumper iCubDev YARP::YARP_init)
INSTALL(TARGETS controlBoardDumper DESTINATION bin)

