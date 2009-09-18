#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

INCLUDE_DIRECTORIES(
	/usr/include/opencv
	/usr/local/include/nnfw
	${PROJECT_SOURCE_DIR}/utils/display 
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	${PROJECT_SOURCE_DIR}/zdfserver
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/display 
	${PROJECT_SOURCE_DIR}/zdfserver
	)

ADD_EXECUTABLE( objRec
		main.cc
		)

TARGET_LINK_LIBRARIES(objRec
        Qt3Support
        QtCore
        QtGui
	display
	ippi
	pthread
	/usr/local/lib/libnnfw.a
	/usr/lib/libcv.so
	/usr/lib/libcvaux.so
	/usr/lib/libcxcore.so
	/usr/lib/libhighgui.so
	)

# install targets
#INSTALL_TARGETS(${BIN_DIR} objRec )
