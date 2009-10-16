#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

INCLUDE_DIRECTORIES(
	/usr/local/include/opencv
	/usr/local/include/nnfw
	${PROJECT_SOURCE_DIR}/zdfserver
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/zdfserver
	)

ADD_EXECUTABLE( objRecServer
		main.cc
		objRec.cc
		)

TARGET_LINK_LIBRARIES(objRecServer
	/usr/local/lib64/libnnfw.a
	/usr/local/lib/libcv.so
	/usr/local/lib/libcvaux.so
	/usr/local/lib/libcxcore.so
	/usr/local/lib/libhighgui.so
	ippi
	pthread
	Qt3Support
        QtCore
	)

# install targets
INSTALL_TARGETS(${BIN_DIR} objRecServer )
