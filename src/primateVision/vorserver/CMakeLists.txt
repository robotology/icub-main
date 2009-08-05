#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */



ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX)


INCLUDE_DIRECTORIES(

	)

LINK_DIRECTORIES(

	)

ADD_EXECUTABLE( vorserver
		main.cc 
		vor.cc
		)

TARGET_LINK_LIBRARIES(vorserver
  Qt3Support
  QtCore
  )

# install targets
INSTALL_TARGETS(${BIN_DIR} 
  vorserver
  )
