#/*
# * Copyright (C) 2009 Stephen Hart. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated)

INCLUDE_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}/resources
        ${PROJECT_SOURCE_DIR}/potentialFunctions
	${PROJECT_SOURCE_DIR}/jacobians
	${PROJECT_SOURCE_DIR}/control
        )

LINK_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}/resources
        ${PROJECT_SOURCE_DIR}/potentialFunctions
	${PROJECT_SOURCE_DIR}/jacobians
        ${PROJECT_SOURCE_DIR}/control
)

ADD_EXECUTABLE( startResources
                startResources.cpp
                )

TARGET_LINK_LIBRARIES(startResources
  cbresources
  cbpf
  cbjacobian
  cbcontrol
        )

#
# install targets                                                                                                                                                               
#INSTALL_TARGETS(${BIN_DIR}
#  controlTest
#  )




