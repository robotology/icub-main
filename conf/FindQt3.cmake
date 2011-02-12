#
# Copyright: 2010 RobotCub Consortium
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 
# Wrap kitware's original FindQt3 script. Standardize varibles.
#
# In windows require you set QTDIR
#
# Set: 
# Qt3_FOUND
# Qt3_LIBRARIES
# Qt3_INCLUDE_DIRS

#message(Find Qt3 form iCub package)

# save current CMAKE_MODULE_PATH, disable it 
# to avoid recursive calls to FindGLUT
set(_CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "")

if (QTDIR)
	message(${QTDIR})
endif(QTDIR)
	
find_package(Qt3)

if (QT_FOUND)
	set(Qt3_INCLUDE_DIRS ${QT_INCLUDE_DIR})
	set(Qt3_LIBRARIES ${QT_LIBRARIES})
	set(Qt3_FOUND TRUE)
endif(QT_FOUND)

# push back original CMAKE_MODULE_PATH
set(CMAKE_MODULE_PATH ${_CMAKE_MODULE_PATH})