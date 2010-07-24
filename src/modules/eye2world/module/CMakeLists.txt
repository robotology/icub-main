set(PROJECTNAME eye2world)
project(${PROJECTNAME})

include_directories(${iKin_INCLUDE_DIRS}
                       ${PROJECT_SOURCE_DIR}/../lib/libvislab/src
					   ${PROJECT_SOURCE_DIR}/../lib/libvislab_YARP/src
					   ${PROJECT_SOURCE_DIR}/../lib/libeyecub/src
					   ${YARP_INCLUDE_DIRS})

set(SOURCE_DIR "src/")

file(GLOB_RECURSE sources "${SOURCE_DIR}*.cpp")
file(GLOB_RECURSE headers "${SOURCE_DIR}*.h")
source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})

add_executable(${PROJECTNAME} ${headers} ${sources})
target_link_libraries(${PROJECTNAME} iKin vislab vislab_YARP eyecub ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)

