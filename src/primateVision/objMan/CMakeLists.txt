#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

SUBDIRS(objManClient)

INCLUDE_DIRECTORIES(
  $ENV{ICUB_DIR}/src/ctrlLib/include/
  ${PROJECT_SOURCE_DIR}/objRec/
	)

LINK_DIRECTORIES(
 $ENV{ICUB_DIR}/lib
	)

ADD_EXECUTABLE( objManServer
		main.cc 
		objMan.cc
		kal.cc
		)

TARGET_LINK_LIBRARIES(objManServer
	ippi
	pthread
        ctrlLib
	Qt3Support
        QtCore
	)

# install targets
INSTALL_TARGETS(${BIN_DIR} objManServer )
