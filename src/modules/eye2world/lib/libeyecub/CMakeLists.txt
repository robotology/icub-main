set(TARGET_NAME eyecub)
project(lib${TARGET_NAME})

include_directories(${PROJECT_SOURCE_DIR}/../libvislab/src
                      ${PROJECT_SOURCE_DIR}/../libvislab_YARP/src
                      ${iKin_INCLUDE_DIRS}
					  ${YARP_INCLUDE_DIRS})

set(SOURCE_DIR "src/")

file(GLOB_RECURSE sources "${SOURCE_DIR}*.cpp")
file(GLOB_RECURSE headers "${SOURCE_DIR}*.h")
source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})

add_library(${TARGET_NAME} ${headers} ${sources})
target_link_libraries(${TARGET_NAME} iKin vislab ${YARP_LIBRARIES})
install(TARGETS ${TARGET_NAME} DESTINATION lib)
