# Copyright: (C) 2013 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Name of the project must correspond to the class name that needs to be 
# instantiated by the factory

SET(COMAN_MC comanMotionControl)
SET(COMAN_FT comanFTsensor)
SET(COMAN_VO comanVelocityObserver)
SET(COMAN_JT comanJointTorqueSensor)


PROJECT(coman)

PREPARE_DEVICE(${COMAN_MC} TYPE ${COMAN_MC} INCLUDE ${COMAN_MC}.h  WRAPPER controlboardwrapper)
PREPARE_DEVICE(${COMAN_FT} TYPE ${COMAN_FT} INCLUDE ${COMAN_FT}.h  WRAPPER analogServer)
PREPARE_DEVICE(${COMAN_VO} TYPE ${COMAN_VO} INCLUDE ${COMAN_VO}.h  WRAPPER analogServer)
PREPARE_DEVICE(${COMAN_JT} TYPE ${COMAN_JT} INCLUDE ${COMAN_JT}.h  WRAPPER analogServer)


if (USE_REALTIME)
  add_definitions(-DRT_ENV)
endif (USE_REALTIME)

set (COMAN_MC_SRC   comanMotionControl.cpp       comanMotionControl.h)
set (COMAN_FT_SRC   comanFTsensor.cpp            comanFTsensor.h )
set (COMAN_VO_SRC   comanVelocityObserver.cpp    comanVelocityObserver.h)
set (COMAN_JT_SRC   comanJointTorqueSensor.cpp   comanJointTorqueSensor.h)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${robolli_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/comanLib  ${YARP_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS} ../debugStream/)

unset(ICUB_COMPILE_COMAN_LIBRARY CACHE)


# useful if we need to add special compilation flags for some target only.
#set( coman_build_flags " -std=c++0x")

IF (NOT SKIP_${COMAN_MC} )
    set(ICUB_COMPILE_COMAN_LIBRARY ON CACHE INTERNAL "use coman library")
    yarp_add_plugin(${COMAN_MC} ${COMAN_MC_SRC} )
#    set_target_properties(${COMAN_MC} PROPERTIES COMPILE_FLAGS ${coman_build_flags})
    target_link_libraries(${COMAN_MC} coman_handler_lib debugStream  iCubDev ${YARP_LIBRARIES})
    icub_export_library(${COMAN_MC} )
ENDIF (NOT SKIP_${COMAN_MC} )


IF (NOT SKIP_${COMAN_FT} )
    set(ICUB_COMPILE_COMAN_LIBRARY ON CACHE INTERNAL "use coman library")
    yarp_add_plugin(${COMAN_FT} ${COMAN_FT_SRC})
#    set_target_properties(${COMAN_FT} PROPERTIES COMPILE_FLAGS ${coman_build_flags})
    target_link_libraries(${COMAN_FT} coman_handler_lib ${COMAN_LIB} debugStream iCubDev ${YARP_LIBRARIES})
    icub_export_library(${COMAN_FT} )
ENDIF (NOT SKIP_${COMAN_FT} )


IF (NOT SKIP_${COMAN_VO} )
    set(ICUB_COMPILE_COMAN_LIBRARY ON CACHE INTERNAL "use coman library")
    yarp_add_plugin(${COMAN_VO} ${COMAN_VO_SRC})
#    set_target_properties(${COMAN_VO} PROPERTIES COMPILE_FLAGS ${coman_build_flags})
    target_link_libraries(${COMAN_VO} coman_handler_lib ${COMAN_LIB} debugStream iCubDev ${YARP_LIBRARIES})
    icub_export_library(${COMAN_VO} )
ENDIF (NOT SKIP_${COMAN_VO} )


IF (NOT SKIP_${COMAN_JT} )
    set(ICUB_COMPILE_COMAN_LIBRARY ON CACHE INTERNAL "use coman library")
    yarp_add_plugin(${COMAN_JT} ${COMAN_JT_SRC})
#    set_target_properties(${COMAN_JT} PROPERTIES COMPILE_FLAGS ${coman_build_flags})
    target_link_libraries(${COMAN_JT} coman_handler_lib ${COMAN_LIB} debugStream iCubDev ${YARP_LIBRARIES})
    icub_export_library(${COMAN_JT} )
ENDIF (NOT SKIP_${COMAN_JT} )
