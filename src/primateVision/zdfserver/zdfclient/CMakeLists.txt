#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/mosaic
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	${PROJECT_SOURCE_DIR}/zdfserver
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/mosaic
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	${PROJECT_SOURCE_DIR}/zdfserver
	)

ADD_EXECUTABLE( zdfclient
		main.cc
		)

TARGET_LINK_LIBRARIES(zdfclient
  QtGui
  Qt3Support
  QtCore
  multiframeviewer
  ippi
  ippcv
  pthread
  )

ADD_EXECUTABLE( zdfclientmos
		zdfmos.cc
		)

TARGET_LINK_LIBRARIES(zdfclientmos
  QtGui
  Qt3Support
  QtCore
  mosaic
  multiframeviewer
  ippi
  ippcv
  pthread
  )


# install targets
INSTALL_TARGETS(${BIN_DIR} 
  zdfclient
  zdfclientmos
  )
