CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
set(PROJECTNAME iCubTest)

file(GLOB folder_header *.h)
file(GLOB folder_source *.cpp)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${iCubDev_INCLUDE_DIRS})
include_directories(${YARP_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
add_library(${PROJECTNAME}Lib ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${iCubDev_LIBRARIES})

install(TARGETS ${PROJECTNAME}Lib DESTINATION lib)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
