SET(PROJECTNAME "camCalib")

PROJECT(${PROJECTNAME})

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${YARP_INCLUDE_DIRS}
    ${SPMAP_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})



# Set up our main executable.
ADD_EXECUTABLE(${PROJECTNAME} 
                ${folder_source} 
                ${folder_header})

target_link_libraries(${PROJECTNAME}
    ${OpenCV_LIBRARIES}
    ${SPMAP_LIBRARIES}
    ${YARP_LIBRARIES})

 INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)