#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

SUBDIRS(testdog)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
	)

LINK_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/utils/convert_bitdepth
	)

ADD_LIBRARY( dog
	     SHARED
	     dog.cc
	     )

TARGET_LINK_LIBRARIES(dog
        Qt3Support
        QtCore
	convert_bitdepth
	)

# install targets
INSTALL_TARGETS(${LIB_DIR} 
  dog
  )
