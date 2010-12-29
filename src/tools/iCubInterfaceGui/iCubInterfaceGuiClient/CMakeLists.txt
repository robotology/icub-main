set(PROJECTNAME iCubInterfaceGuiClient)

add_definitions(${GtkMM_C_FLAGS})
add_definitions(${Gthread_C_FLAGS})

file(GLOB folder_header *.h)
file(GLOB folder_source *.cpp)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../iCubInterfaceGuiLib)
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${iCubDev_INCLUDE_DIRS})
include_directories(${GtkMM_INCLUDE_DIRS})
include_directories(${Gthread_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} 
                      ${GtkMM_LIBRARIES} 
                      ${YARP_LIBRARIES} 
                      ${Gthread_LIBRARIES}
		      iCubInterfaceGuiLib)

install(TARGETS ${PROJECTNAME} DESTINATION bin)
