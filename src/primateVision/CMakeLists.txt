#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

PROJECT(primateVision)

FIND_PACKAGE(YARP)
FIND_PACKAGE(OpenCV)
FIND_PACKAGE(ICUB)

INCLUDE_DIRECTORIES(/opt/intel/ipp/include
                    $ENV{QT_INCLUDE_DIR}
                    $ENV{QT_INCLUDE_DIR}/Qt
                    $ENV{QT_INCLUDE_DIR}/Qt3Support
                    ${OPENCV_INCLUDE_DIR}
                   )

LINK_DIRECTORIES(/opt/intel/ipp/sharedlib
                )


#When we type "make install"
SET(BIN_DIR /bin)
SET(LIB_DIR /lib)
SET(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})


SUBDIRS(
        armExtractor
	utils 
	svogrid
	egoSphere2
	recserver 
	vorserver
	zdfserver
	sptzdfserver
	nzdfserver
	colcsserver
	ycsserver
	flowserver
	depthflowserver
	ocsserver
	tsbserver
	iorserver
	salserver
	attentionserver
	attentionserver_stereo
	calmot
	tempTrackSimple
	colTrackSimple
        CVDepthClient2
	objRec
	objMan
        demos
	devel
	)
