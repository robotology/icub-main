# Copyright: (C) 2013 Isituto Italiano di Tecnologia iCub Facility
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

#
# demoModule CMake file.
#
set(PROJECTNAME "rpcIdl")
project(${PROJECTNAME})

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS}   
)


include(YarpIDL)

set(generated_libs_dir "${CMAKE_CURRENT_SOURCE_DIR}")
yarp_idl_to_dir(IRpcServer.thrift ${generated_libs_dir})

# Search for source code.
file(GLOB folder_source src/*.cpp src/*.cc src/*.c)
file(GLOB folder_header include/iCub/*.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

add_executable(${PROJECTNAME} 
   ${folder_source}
   ${folder_header}
)

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES}) 

install(TARGETS ${PROJECTNAME} DESTINATION bin)

yarp_install(FILES ${conf} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname})

