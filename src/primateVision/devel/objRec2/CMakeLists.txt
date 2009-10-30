#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/display
	${PROJECT_SOURCE_DIR}/utils/multiFrameViewer
	${PROJECT_SOURCE_DIR}/zdfserver
	${PROJECT_SOURCE_DIR}/ocsserver
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/display
	${PROJECT_SOURCE_DIR}/zdfserver
	${PROJECT_SOURCE_DIR}/ocsserver
	)

ADD_EXECUTABLE( objRec2
		main.cc
		)

TARGET_LINK_LIBRARIES(objRec2
  Qt3Support
  QtCore
  QtGui
  display
  pthread
  )


# install targets
#INSTALL_TARGETS(${BIN_DIR} 
#  objRec2
#  )
