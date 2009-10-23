#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)


INCLUDE_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}/utils/timing
        ${PROJECT_SOURCE_DIR}/utils/coord
	${PROJECT_SOURCE_DIR}/utils/display
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	${PROJECT_SOURCE_DIR}/utils/convert_rgb
	${PROJECT_SOURCE_DIR}/utils/multiclass
	${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
	)

LINK_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}/utils/timing
	${PROJECT_SOURCE_DIR}/utils/display
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	${PROJECT_SOURCE_DIR}/utils/convert_rgb
	${PROJECT_SOURCE_DIR}/utils/multiclass
	${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
	)

ADD_EXECUTABLE( body
		main.cpp
		)

TARGET_LINK_LIBRARIES(body
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
	convert_bitdepth
	timing
	)

# install targets
INSTALL_TARGETS(${BIN_DIR} 
  body
)