set(PROJECTNAME yuvProcessor)
project(${PROJECTNAME})

# Search for source code.
file(GLOB folder_source src/*.cpp src/*.cc src/*.c)
file(GLOB folder_header include/iCub/*.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS}
	${ICUB_INCLUDE_DIRS}
	${IPP_INCLUDE_DIRS}
)

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(
    ${PROJECTNAME}
	${IPP_LIBRARIES}
	${YARP_LIBRARIES}
)

install(TARGETS ${PROJECTNAME} DESTINATION bin)

