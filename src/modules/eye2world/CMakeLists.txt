set(PROJECTNAME eye2world)
project(${PROJECTNAME})

include_directories(${iKin_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include
		    	    ${YARP_INCLUDE_DIRS})

file(GLOB_RECURSE sources "src/*.cpp")
file(GLOB_RECURSE headers "include/iCub/*.h")
source_group("Source Files" FILES ${sources})
source_group("Header Files" FILES ${headers})

add_executable(${PROJECTNAME} ${headers} ${sources})
target_link_libraries(${PROJECTNAME} iKin ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)
