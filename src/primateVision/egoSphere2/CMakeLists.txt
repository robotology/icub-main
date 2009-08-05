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
	)

ADD_EXECUTABLE( egosphere2
		main.cc
		world3d.cc
 		mworld3d.cc
		proc.cc
		)

TARGET_LINK_LIBRARIES(egosphere2
	ippi
	glut
	Qt3Support
        QtCore
        QtOpenGL
        QtGui
	)

# install targets
INSTALL_TARGETS(${BIN_DIR} 
  egosphere2
  )
