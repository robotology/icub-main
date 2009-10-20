#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/mosaic
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	${PROJECT_SOURCE_DIR}/demos/objMan
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/mosaic
	${PROJECT_SOURCE_DIR}/demos/objMan
	)

ADD_EXECUTABLE( objManClient
		main.cc 
		)

TARGET_LINK_LIBRARIES(objManClient
	ippi
	pthread
        Qt3Support
        QtCore
        QtGui
	mosaic
	pthread	
	)

# install targets
INSTALL_TARGETS(${BIN_DIR} objManClient )
