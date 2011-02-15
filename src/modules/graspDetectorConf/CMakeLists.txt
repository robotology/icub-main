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

SET(PROJECTNAME graspDetectorConf)

PROJECT(${PROJECTNAME})

SET(folder_source main.cpp
                  graspDetectorConf.cpp)

SET(folder_header graspDetectorConf.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${GSL_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${GSL_LIBRARIES}
                                     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

