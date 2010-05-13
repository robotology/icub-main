SET(PROJECTNAME velocityObserver)
PROJECT(${PROJECTNAME})

SET(folder_source main.cpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})

include_directories(${ctrlLib_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${folder_source})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})
target_link_libraries(${PROJECTNAME} ctrlLib)
install(TARGETS ${PROJECTNAME} DESTINATION bin)



