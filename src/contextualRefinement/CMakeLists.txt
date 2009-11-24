SET(KEYWORD "contextualRefinement")

PROJECT(${KEYWORD})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(IPP REQUIRED)

#--------------------------
#FIND_PACKAGE(LOGPOLAR REQUIRED)
#LINK_LIBRARIES(${LOGPOLAR_LIBRARIES})
#INCLUDE_DIRECTORIES(${LOGPOLAR_INCLUDE_DIRS})
#--------------------------


#search for includes
INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${OPENCV_INCLUDE_DIR}
    ${IPP_INCLUDE_DIRS}	 
    E:/work2/primateVision/utils/multiclass
    E:/work2/primateVision/utils/coord
)

IF(NOT IPP_INCLUDE_DIRS)
	MESSAGE(ERROR ": No IPP_INCLUDE_DIR found. Please check it out")	
ENDIF(NOT IPP_INCLUDE_DIRS)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})


LINK_DIRECTORIES(
    ${ICUB_SOURCE_DIR}
)


LINK_LIBRARIES(
    ${OPENCV_LIBRARIES}
    ${IPP_LIBRARIES} 
    E:/work2/primateVision/utils/multiclass/debug/multiclass
)



# Set up our main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )	
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)


INSTALL_TARGETS(/bin ${KEYWORD})

