#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */


ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/timing
	${PROJECT_SOURCE_DIR}/utils/depth
	${PROJECT_SOURCE_DIR}/utils/dog
	${PROJECT_SOURCE_DIR}/utils/coord
	${PROJECT_SOURCE_DIR}/utils/norm
	${PROJECT_SOURCE_DIR}/utils/display
	${PROJECT_SOURCE_DIR}/utils/convert_rgb
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/timing
	${PROJECT_SOURCE_DIR}/utils/depth
	${PROJECT_SOURCE_DIR}/utils/dog
	${PROJECT_SOURCE_DIR}/utils/norm
	${PROJECT_SOURCE_DIR}/utils/display
	${PROJECT_SOURCE_DIR}/utils/convert_rgb
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	)
 
ADD_EXECUTABLE( armX
		main.cpp
		armXopt.cpp
		graph.cpp
		maxflow.cpp
		)

TARGET_LINK_LIBRARIES(armX
	display
	multiframeviewer
	convert_rgb
	ippi
	norm
	timing
	depth
	dog
	Qt3Support
        QtCore
        QtGui
        ${OPENCV_LIBRARIES}
	)


# install targets
INSTALL_TARGETS(${BIN_DIR} 
  armX
  )
