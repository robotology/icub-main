#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/display
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	${PROJECT_SOURCE_DIR}/nzdfserver
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/display
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	${PROJECT_SOURCE_DIR}/nzdfserver
	)

ADD_EXECUTABLE( nzdfclient
		main.cc
		)

TARGET_LINK_LIBRARIES(nzdfclient
       QtGui
       Qt3Support
        QtCore
	display
	multiframeviewer
	ippi
	ippcv
        pthread
	)


# install targets
INSTALL_TARGETS(${BIN_DIR} 
  nzdfclient
  )
