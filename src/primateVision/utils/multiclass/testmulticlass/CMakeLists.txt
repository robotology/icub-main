#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)


INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/display
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	${PROJECT_SOURCE_DIR}/utils/convert_rgb
	${PROJECT_SOURCE_DIR}/utils/canny 
	${PROJECT_SOURCE_DIR}/utils/multiclass
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/display
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	${PROJECT_SOURCE_DIR}/utils/convert_rgb
	${PROJECT_SOURCE_DIR}/utils/canny
	${PROJECT_SOURCE_DIR}/utils/multiclass
	)

ADD_EXECUTABLE( testmulticlass
		main.cpp
		)

TARGET_LINK_LIBRARIES(testmulticlass
        Qt3Support
        QtCore
        QtGui
	display
	ippi
	ippcv
	ippcc
	display
	multiframeviewer
	convert_rgb
	multiclass
        pthread
	canny
	)

# install targets
INSTALL_TARGETS(${BIN_DIR} 
  testmulticlass
  )
