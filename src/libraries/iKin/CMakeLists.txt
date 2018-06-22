# Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
# Copyright (C) 2006-2010 RobotCub Consortium
# All rights reserved.
#
# This software may be modified and distributed under the terms
# of the BSD-3-Clause license. See the accompanying LICENSE file for
# details.

project(iKin)

set(folder_source src/iKinFwd.cpp
                  src/iKinInv.cpp
                  src/iKinHlp.cpp)

set(folder_header include/iCub/iKin/iKinFwd.h
                  include/iCub/iKin/iKinInv.h
                  include/iCub/iKin/iKinVocabs.h
                  include/iCub/iKin/iKinHlp.h)

if(ICUB_USE_IPOPT)
   set(folder_source ${folder_source}
                     src/iKinIpOpt.cpp
                     src/iKinSlv.cpp)

   set(folder_header ${folder_header}
                     include/iCub/iKin/iKinIpOpt.h
                     include/iCub/iKin/iKinSlv.h)   
endif()

if(ICUB_USE_IPOPT)
   add_definitions(${IPOPT_DEFINITIONS})
endif()

add_definitions(-D_USE_MATH_DEFINES)
add_library(${PROJECT_NAME} ${folder_source} ${folder_header})

if(ICUB_USE_IPOPT)
  target_include_directories(${PROJECT_NAME} PUBLIC  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
                                                     "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>"
                                             PRIVATE ${IPOPT_INCLUDE_DIRS})
else()
  target_include_directories(${PROJECT_NAME} PUBLIC  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
                                                     "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")
endif()

target_link_libraries(${PROJECT_NAME} ctrlLib ${YARP_LIBRARIES})

if(ICUB_USE_IPOPT)
   set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " ${IPOPT_LINK_FLAGS}")
   target_link_libraries(${PROJECT_NAME} ${IPOPT_LIBRARIES})
endif()

icub_export_library(${PROJECT_NAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                    DEPENDS ctrlLib
                                    DESTINATION include/iCub/iKin
                                    FILES ${folder_header})                                    


