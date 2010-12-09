CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
set(PROJECTNAME iCubInterfaceGuiClient)

add_definitions(${GTKMM_C_FLAGS})

file(GLOB folder_header *.h)
file(GLOB folder_source *.cpp)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../iCubInterfaceGuiLib)
include_directories(${YARP_INCLUDE_DIRS})
#include_directories(${iCubDev_INCLUDE_DIRS})
include_directories(${GTKMM_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} 
                      ${GTKMM_LIBRARIES} 
                      ${YARP_LIBRARIES} 
                      ${GTHREAD_LIBRARIES}
		              iCubInterfaceGuiLib)

install(TARGETS ${PROJECTNAME} DESTINATION bin)
