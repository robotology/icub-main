#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/depth
	${PROJECT_SOURCE_DIR}/utils/display
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	${PROJECT_SOURCE_DIR}/utils/mmx
	${PROJECT_SOURCE_DIR}/utils/sptmp_mmx
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/depth
	)

ADD_EXECUTABLE( testdepth
		main.cc
		)

TARGET_LINK_LIBRARIES(testdepth
        Qt3Support
        QtCore
	ippi
	pthread
	depth
	display
	)

