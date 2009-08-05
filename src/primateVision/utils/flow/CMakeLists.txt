#/*
# * Copyright (C) 2003-2008 Andrew Dankers. ALl rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

SUBDIRS(
	testflow
	testflowlive
	)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/sptmp_MMX
	${PROJECT_SOURCE_DIR}/utils/mmx 
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/sptmp_MMX
	)

ADD_LIBRARY( flow
	     STATIC
	     flow.cc
	     )

TARGET_LINK_LIBRARIES(flow
        Qt3Support
        QtCore
	ippi
	sptmpmmx
        mmx
	)


# install targets
INSTALL_TARGETS(${LIB_DIR} 
  flow
  )
