#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/centsur
	${PROJECT_SOURCE_DIR}/utils/display
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	${PROJECT_SOURCE_DIR}/utils/timing
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/centsur
	${PROJECT_SOURCE_DIR}/utils/display
	${PROJECT_SOURCE_DIR}/utils/timing
	)

ADD_EXECUTABLE( testcentsur
		main.cc
		)

TARGET_LINK_LIBRARIES(testcentsur
        Qt3Support
        QtCore
	ippcv
	pthread
	centsur
	display
	timing
	)

