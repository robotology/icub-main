#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)


INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/recserver
	           )

ADD_EXECUTABLE( CVDepthClient2
		main.cpp
		)

TARGET_LINK_LIBRARIES(CVDepthClient2
        ${OPENCV_LIBRARIES}
        ippi
        pthread
	)

# install targets
INSTALL_TARGETS(${BIN_DIR} 
        CVDepthClient2
  )
