#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

SUBDIRS(salclient)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/flowserver 
	${PROJECT_SOURCE_DIR}/ycsserver
	${PROJECT_SOURCE_DIR}/colcsserver  
	${PROJECT_SOURCE_DIR}/ocsserver  
	${PROJECT_SOURCE_DIR}/utils/timing
	${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
	) 

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/timing
	${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
	)

ADD_EXECUTABLE( salserver
		main.cc
		sal.cc
		) 

TARGET_LINK_LIBRARIES(salserver 
        Qt3Support
        QtCore
	ippi
	timing
	convert_bitdepth
	)



# install targets
INSTALL_TARGETS(${BIN_DIR} 
  salserver
  )
