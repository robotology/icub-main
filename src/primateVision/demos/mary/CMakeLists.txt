#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/display 
	${PROJECT_SOURCE_DIR}/utils/convert_rgb
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	${PROJECT_SOURCE_DIR}/attentionserver_stereo
	${PROJECT_SOURCE_DIR}/recserver
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/display 
	${PROJECT_SOURCE_DIR}/utils/convert_rgb
	)

ADD_EXECUTABLE( mary
		main.cc
		)

TARGET_LINK_LIBRARIES(mary
        Qt3Support
        QtCore
        QtGui
	display
	ippi
	pthread
	convert_rgb
	)

# install targets
INSTALL_TARGETS(${BIN_DIR} 
  mary
  )
