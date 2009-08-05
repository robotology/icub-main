#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

SUBDIRS(depthflowclient)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/recserver  
	${PROJECT_SOURCE_DIR}/utils/timing
	${PROJECT_SOURCE_DIR}/utils/depth
	${PROJECT_SOURCE_DIR}/utils/depthflow
	${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
        ) 

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/recserver  
	${PROJECT_SOURCE_DIR}/utils/timing
	${PROJECT_SOURCE_DIR}/utils/depth
	${PROJECT_SOURCE_DIR}/utils/depthflow
	${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
	)

ADD_EXECUTABLE( depthflowserver
		main.cc
		depthflows.cc
		) 

TARGET_LINK_LIBRARIES(depthflowserver 
	ippi
	depthflow
	timing
        Qt3Support
        QtCore
        convert_bitdepth
	)

# install targets
INSTALL_TARGETS(${BIN_DIR} 
  depthflowserver
  )
