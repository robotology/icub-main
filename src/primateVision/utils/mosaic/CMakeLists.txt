#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

INCLUDE_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
        ${PROJECT_SOURCE_DIR}/utils/display
        ${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
	)
LINK_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
        ${PROJECT_SOURCE_DIR}/utils/display
        ${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
	)

ADD_LIBRARY(	mosaic
		SHARED
		mosaic.cc
		)

TARGET_LINK_LIBRARIES(	mosaic
                        Qt3Support
                        QtCore
			multiframeviewer
			convert_bitdepth
			)


# install targets
INSTALL_TARGETS(${LIB_DIR} 
  mosaic
  )
