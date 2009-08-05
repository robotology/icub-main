#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/sptmp_MMX
	${PROJECT_SOURCE_DIR}/utils/mmx
	${PROJECT_SOURCE_DIR}/utils/display
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	${PROJECT_SOURCE_DIR}/utils/dog
	${PROJECT_SOURCE_DIR}/utils/flow
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/sptmp_MMX
	${PROJECT_SOURCE_DIR}/utils/display
	${PROJECT_SOURCE_DIR}/utils/dog
	${PROJECT_SOURCE_DIR}/utils/flow
	)

ADD_EXECUTABLE( testflow
		main.cc
		)

TARGET_LINK_LIBRARIES(testflow
        Qt3Support
        QtCore
	ippi
	pthread
	flow
	display
	sptmpmmx
	)

