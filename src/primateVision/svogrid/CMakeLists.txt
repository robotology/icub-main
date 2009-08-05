#/*
# * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated -D_LINUX -DQT_THREAD_SUPPORT -DQT3_SUPPORT)


ADD_CUSTOM_COMMAND(  OUTPUT mworld3d.cc
                     COMMAND moc-qt4 world3d.h -o mworld3d.cc
                     DEPENDS world3d.cc world3d.h)


INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/recserver
	${PROJECT_SOURCE_DIR}/flowserver
	${PROJECT_SOURCE_DIR}/depthflowserver
	)

ADD_EXECUTABLE( svogrid
		main.cc
		world3d.cc
 		mworld3d.cc
		svogrid.cc
		)

TARGET_LINK_LIBRARIES(svogrid
	glut
	Qt3Support
        QtCore
        QtOpenGL
        QtGui
	ippi
	ippcc
	)

# install targets
INSTALL_TARGETS(${BIN_DIR} 
  svogrid
  )
