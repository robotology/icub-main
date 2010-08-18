set(PROJECTNAME logpolarTransform)
project(${PROJECTNAME})

file(GLOB folder_source src/*.cpp src/*.cc src/*.c)
file(GLOB folder_header include/iCub/*.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${logPolar_INCLUDE_DIRS}
    ${YARP_INCLUDE_DIRS}
    #${iCubVis_INCLUDE_DIRS}	
    #${FOURIERVISION_INCLUDE_DIRS}
)

#message("iCubVis_INCLUDE_DIRS: ${iCubVis_INCLUDE_DIRS} ")
#message("iCubVis_LIBRARIES: ${iCubVis_LIBRARIES} ")
#message("iKin_LIBRARIES: ${iKin_LIBRARIES}")
#iCubVis

if (folder_source)
	add_executable(${PROJECTNAME} ${folder_source} ${header_files})
	target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} logPolar) 
	install(TARGETS ${PROJECTNAME} DESTINATION bin)
else (folder_source)
    message(STATUS "No source code files found. Please add something")
endif (folder_source)

