SET(KEYWORD "biologicalControlGazeInterface")

PROJECT(${KEYWORD})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(ODE REQUIRED)
#FIND_PACKAGE(IPL REQUIRED)
#FIND_PACKAGE(CVBLOBS REQUIRED)
FIND_PACKAGE(IPP REQUIRED)


#--------------------------------
FIND_PACKAGE(BML REQUIRED)
LINK_LIBRARIES(${BML_LIBRARIES})
INCLUDE_DIRECTORIES(${BML_INCLUDE_DIRS})
#--------------------------


#---------------------------------------
FIND_PACKAGE(GtkPlus REQUIRED)
ADD_DEFINITIONS(${GTKPLUS_C_FLAGS})
LINK_LIBRARIES(${GTKPLUS_LINK_FLAGS})
INCLUDE_DIRECTORIES(${GTKPLUS_INCLUDE_DIR})
#---------------------------------------


#search for includes
INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${OPENCV_INCLUDE_DIR}    
    ${ODE_INCLUDE_DIRS}	
    ${IPP_INCLUDE_DIRS}
    #${IPL_INCLUDE_DIRS}	
    #${CVBLOBS_INCLUDE_DIRS}
)


# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})



LINK_DIRECTORIES(
    ${ICUB_SOURCE_DIR}
    #${CVBLOBS_SOURCE_DIRS}	
)


LINK_LIBRARIES(
    ${OPENCV_LIBRARIES}
    #${CVBLOBS_LIBRARIES}
    ${ODE_LIBRARIES}
    ${IDL_LIBRARIES}
    ${IPP_LIBRARIES}	 
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

