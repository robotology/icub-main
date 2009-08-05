#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/display 
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	${PROJECT_SOURCE_DIR}/flowserver
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/display 
	${PROJECT_SOURCE_DIR}/flowserver
	)

ADD_EXECUTABLE( flowclient
		main.cc
		)

TARGET_LINK_LIBRARIES(flowclient
        Qt3Support
        QtCore
        QtGui
	display
	pthread
	)

# install targets
INSTALL_TARGETS(${BIN_DIR} 
  flowclient
  )
