#/*
# * Copyright (C) 2009 Stephen Hart. All rights reserved.
# * 
# */

ADD_DEFINITIONS(-O3 -Wno-deprecated)

FIND_PACKAGE(GtkPlus)
IF (NOT GtkPlus_FOUND)
    MESSAGE(ERROR "gtk+ not found, cannot compile the application!")
ENDIF (NOT GtkPlus_FOUND)

FIND_PACKAGE(GtkMM REQUIRED)

ADD_DEFINITIONS(${GTKPLUS_C_FLAGS})
ADD_DEFINITIONS(${GTKMM_C_FLAGS})
LINK_LIBRARIES(${GTKPLUS_LINK_FLAGS})
LINK_LIBRARIES(${GTKMM_LINK_FLAGS})

INCLUDE_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}/resources
        ${PROJECT_SOURCE_DIR}/potentialFunctions
	${PROJECT_SOURCE_DIR}/jacobians
	${PROJECT_SOURCE_DIR}/control
        ${PROJECT_SOURCE_DIR}/util/kinematics
        )

LINK_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}/resources
        ${PROJECT_SOURCE_DIR}/potentialFunctions
	${PROJECT_SOURCE_DIR}/jacobians
        ${PROJECT_SOURCE_DIR}/control
        ${PROJECT_SOURCE_DIR}/util/kinematics
)

ADD_EXECUTABLE( controlBasisGUI               
		controlBasisGUI.cpp
		CBAPIUtil.cpp
		CBAPIHelper.cpp
		)

ADD_EXECUTABLE(referenceGUI
		referenceGUI.cpp
		)

TARGET_LINK_LIBRARIES( referenceGUI
                       cbresources
		       ${GTKMM_LINK_FLAGS}
)

TARGET_LINK_LIBRARIES(controlBasisGUI
  ${IKIN_LIBRARIES}
  cbresources
  cbpf
  cbjacobian
  cbcontrol
  ${GTKMM_LINK_FLAGS}
        )
#
# install targets                                                                                                                                                               
#INSTALL_TARGETS(${BIN_DIR}
#  controlBasisGuI
#  referenceGUI
#  )




