PROJECT(iCubDemoY3)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${YARP_INCLUDE_DIRS})

add_executable(iCubDemoY3 ${folder_source} ${folder_header})

target_link_libraries(iCubDemoY3 ${YARP_LIBRARIES})

install(TARGETS iCubDemoY3 DESTINATION bin)

