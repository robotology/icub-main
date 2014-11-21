# Copyright: (C) 2013 Isituto Italiano di Tecnologia iCub Facility
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

#
# demoModule CMake file.
#
set(PROJECTNAME rpcIdl)
project(${PROJECTNAME})

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS}   
)

include(YarpIDL)

set(generated_libs_dir "${CMAKE_CURRENT_SOURCE_DIR}")
yarp_idl_to_dir(IRpcServer.thrift ${generated_libs_dir})

# Search for source code.
file(GLOB folder_source src/*.cpp)
file(GLOB folder_header include/*.h)
file(GLOB idl_files *.thrift)
set(doc_files ${PROJECTNAME}.xml)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})
source_group("IDL Files"    FILES ${idl_files})
source_group("DOC Files"    FILES ${doc_files})

add_executable(${PROJECTNAME} 
   ${folder_source}
   ${folder_header}
   ${idl_files}
   ${doc_files}
)

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES}) 
install(TARGETS ${PROJECTNAME} DESTINATION bin)
yarp_install(FILES ${doc_files} DESTINATION ${ICUB_MODULES_INSTALL_DIR})

