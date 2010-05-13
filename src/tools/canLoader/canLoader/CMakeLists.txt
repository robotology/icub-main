set(PROJECTNAME canLoader)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../canLoaderLib)
include_directories(${GtkPlus_INCLUDE_DIRS})
include_directories(${YARP_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})

#link_directories(${PROJECTNAME} ${GTKPLUS_LIBRARY_DIR})
target_link_libraries(${PROJECTNAME} canLoaderLib ${YARP_LIBRARIES})
target_link_libraries(${PROJECTNAME} ${GtkPlus_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)

