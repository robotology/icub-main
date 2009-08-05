#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */


ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

SUBDIRS(nzdfclient)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/timing
	${PROJECT_SOURCE_DIR}/utils/depth
	${PROJECT_SOURCE_DIR}/utils/dog
	${PROJECT_SOURCE_DIR}/utils/coord
	${PROJECT_SOURCE_DIR}/utils/norm
	${PROJECT_SOURCE_DIR}/recserver
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/timing
	${PROJECT_SOURCE_DIR}/utils/depth
	${PROJECT_SOURCE_DIR}/utils/dog
	${PROJECT_SOURCE_DIR}/utils/norm
	)
 
ADD_EXECUTABLE( nzdfserver
		main.cc
		nzdf.cc
		nzdfopt.cpp
		graph.cpp
		maxflow.cpp
		)

TARGET_LINK_LIBRARIES(nzdfserver
	ippi
	norm
	timing
	depth
	dog
	Qt3Support
        QtCore
        ${OPENCV_LIBRARIES}
	)



# install targets
INSTALL_TARGETS(${BIN_DIR} 
  nzdfserver
  )
