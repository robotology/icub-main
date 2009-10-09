#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)

INCLUDE_DIRECTORIES(

	)

LINK_DIRECTORIES(

	)

ADD_EXECUTABLE( objMan
		main.cc
		)

TARGET_LINK_LIBRARIES(objMan
	ippi
	)

# install targets
INSTALL_TARGETS(${BIN_DIR} objMan )
