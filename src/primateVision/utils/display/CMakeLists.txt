#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

SUBDIRS(testdisplay)

INCLUDE_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
        ${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
        ${PROJECT_SOURCE_DIR}/utils/norm
	)
LINK_DIRECTORIES(
        ${PROJECT_SOURCE}/utils/multiFrameViewer
        ${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
        ${PROJECT_SOURCE_DIR}/utils/norm
	)

ADD_LIBRARY(	display
		SHARED
		display.cc
		)

TARGET_LINK_LIBRARIES(	display
                        Qt3Support
                        QtCore
                        QtGui
			multiframeviewer
			ippi
			convert_bitdepth
			norm
			)

# install targets
INSTALL_TARGETS(${LIB_DIR} 
  display
  )
