#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

SUBDIRS(iorclient)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/recserver 
	${PROJECT_SOURCE_DIR}/flowserver 
	${PROJECT_SOURCE_DIR}/utils/timing
	${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
	) 

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/timing
	${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
	)

ADD_EXECUTABLE( iorserver
		main.cc
		ior.cc
		) 

TARGET_LINK_LIBRARIES(iorserver 
	ippi
        Qt3Support
        QtCore
	timing
	convert_bitdepth
	)

# install targets
INSTALL_TARGETS(${BIN_DIR} 
  iorserver
  )
