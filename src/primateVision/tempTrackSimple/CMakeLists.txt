#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/convert_rgb
	${PROJECT_SOURCE_DIR}/utils/display
	${PROJECT_SOURCE_DIR}/utils/mydraw
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/convert_rgb
	${PROJECT_SOURCE_DIR}/utils/display
	${PROJECT_SOURCE_DIR}/utils/mydraw
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	)

ADD_EXECUTABLE( tempTrackSimple
		main.cc 
		)

TARGET_LINK_LIBRARIES(tempTrackSimple
        Qt3Support
        QtCore
        QtGui
	ippi
	convert_rgb
	display
	multiframeviewer
        pthread
	ncurses
	mydraw
	)

# install targets
INSTALL_TARGETS(${BIN_DIR} 
  tempTrackSimple
  )
