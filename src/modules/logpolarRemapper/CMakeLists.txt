set(PROJECTNAME logpolarRemapper)
project(${PROJECTNAME})

file(GLOB folder_source src/*.cpp)
file(GLOB header_files include/iCub/*.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${header_files})

#add local directory 
include_directories(${iCubDev_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${folder_source} ${header_files})

#add yarp if needed, no need to call FIND_PACAKGE()
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} iCubDev icubmod) 

install(TARGETS ${PROJECTNAME} DESTINATION bin)
