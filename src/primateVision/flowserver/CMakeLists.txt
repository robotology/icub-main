#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

SUBDIRS(flowclient)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/recserver 
	${PROJECT_SOURCE_DIR}/utils/dog
	${PROJECT_SOURCE_DIR}/utils/timing
	${PROJECT_SOURCE_DIR}/utils/flow
	${PROJECT_SOURCE_DIR}/utils/sptmp_MMX
	${PROJECT_SOURCE_DIR}/utils/mmx
	${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
	) 

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/dog
	${PROJECT_SOURCE_DIR}/utils/timing
	${PROJECT_SOURCE_DIR}/utils/flow
	${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
	)

ADD_EXECUTABLE( flowserver
		main.cc
		flows.cc
		) 

TARGET_LINK_LIBRARIES(flowserver 
	ippi
	ippcv
	dog
	flow
        Qt3Support
        QtCore
	timing
	convert_bitdepth
	)

# install targets
INSTALL_TARGETS(${BIN_DIR} 
  flowserver
  )
