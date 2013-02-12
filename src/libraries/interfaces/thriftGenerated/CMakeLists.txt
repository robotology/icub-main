# Copyright: (C) 2012 Istituto Italiano di Tecnologia (IIT)
# Authors: Elena Ceseracciu
# CopyPolicy: Released under the terms of the GNU GPL v2.0 and later.

set(PROJECTNAME thriftGenerated)
project(${PROJECTNAME})

set(THRIFT_HEADERS)
set(THRIFT_SOURCES)

set(fileList iCub_velocityControl_VelocityControlInterface_thrift.cmake
			 iCub_iSpeak_iSpeakThrift_thrift.cmake)

foreach(scriptFile ${fileList})
  include(${scriptFile})
  list(APPEND THRIFT_HEADERS ${headers})
  list(APPEND THRIFT_SOURCES ${sources})
endforeach()
message ("Thrift headers ${THRIFT_HEADERS}.\n Thirft sources: ${THRIFT_SOURCES}")
include_directories(${PROJECT_SOURCE_DIR}/include ${YARP_INCLUDE_DIRS})

add_library(${PROJECTNAME} ${THRIFT_HEADERS} ${THRIFT_SOURCES})
set_target_properties(${PROJECTNAME} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})

#set(THRIFT_INCLUDE_DIRS "")
#foreach(thr_file ${THRIFT_HEADERS})
#get_filename_component(thr_incl_dir ${thr_file} PATH)
#set(THRIFT_INCLUDE_DIRS ${THRIFT_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/${thr_incl_dir})
#endforeach()
#list(REMOVE_DUPLICATES THRIFT_INCLUDE_DIRS)
#message("thrift include dirs: ${THRIFT_INCLUDE_DIRS}")

icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include  #should add ${YARP_INCLUDE_DIRS} as EXTERNAL_INCLUDE_DIRS ...?
                                   DESTINATION ./ # THRIFT_HEADERS already have the include/iCub/etc path
                                   FILES_WITH_PATH ${THRIFT_HEADERS} VERBOSE)

