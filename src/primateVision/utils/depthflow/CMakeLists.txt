#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)


INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/depth
	)

LINK_DIRECTORIES(
	)

ADD_LIBRARY( depthflow
	     depthflow.cc
	     )

TARGET_LINK_LIBRARIES(depthflow
        Qt3Support
        QtCore
	ippi
        depth
        ${OPENCV_LIBRARIES}
	)


# install targets
INSTALL_TARGETS(${LIB_DIR} 
  depthflow
  )
