# Copyright: (C) 2012 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Name of the project must correspond to the class name that needs to be 
# instantiated by the factory

SET(THIS_LIB comanMotionControl)
PROJECT(${THIS_LIB})

PREPARE_DEVICE(${THIS_LIB} TYPE ${THIS_LIB} INCLUDE ${THIS_LIB}.h  WRAPPER controlboard)

IF (NOT SKIP_${THIS_LIB} )

  if (USE_REALTIME) 
    add_definitions(-DRT_ENV)
  endif (USE_REALTIME)

  #set(CMAKE_INCLUDE_CURRENT_DIR ON)

  set(ThisLib_SOURCES 
  Boards_iface.cpp
  CommProtocol.cpp
  DSP_board.cpp
  comanMotionControl.cpp
  )

  set(HEADERS_TO_INSTALL
  Boards_iface.h
  CommProtocol.hpp
  definitions.h
  DSP_board.h
  comanMotionControl.h
  )

  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS} ../debugStream/)
  #add_library(${THIS_LIB} STATIC ${ThisLib_SOURCES})
  add_library(${THIS_LIB} ${ThisLib_SOURCES})
  target_link_libraries(${THIS_LIB} yaml-cpp debugStream ${YARP_LIBRARIES} ${ACE_LIBRARIES} )

#   install(TARGETS ${THIS_LIB} ARCHIVE DESTINATION lib)
#   install(FILES ${HEADERS_TO_INSTALL} DESTINATION include)

  icub_export_library(${THIS_LIB} )
ENDIF (NOT SKIP_${THIS_LIB} )