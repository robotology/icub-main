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

add_library(${PROJECT_NAME} ${folder_source} ${folder_header})
add_library(ICUB::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
if(ICUB_USE_IPOPT)
   target_compile_definitions(${PROJECT_NAME} PRIVATE ${IPOPT_DEFINITIONS})
endif()
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")
if(ICUB_USE_IPOPT)
  target_include_directories(${PROJECT_NAME} PRIVATE ${IPOPT_INCLUDE_DIRS})
endif()

target_link_libraries(${PROJECT_NAME} ctrlLib ${YARP_LIBRARIES})

set(IKIN_DEPENDENCIES  "ctrlLib HINTS \"\${CMAKE_CURRENT_LIST_DIR}/..\""
                       YARP_os
                       YARP_sig
                       YARP_dev
                       YARP_math)

if(ICUB_USE_IPOPT)
   set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " ${IPOPT_LINK_FLAGS}")
   target_link_libraries(${PROJECT_NAME} ${IPOPT_LIBRARIES})
   list(APPEND IKIN_DEPENDENCIES IPOPT)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES
                                      PUBLIC_HEADER "${folder_header}")

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        LIBRARY
            DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        PUBLIC_HEADER
            DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/iCub/iKin")


icub_install_basic_package_files(${PROJECT_NAME}
                                 DEPENDENCIES ${IKIN_DEPENDENCIES})
