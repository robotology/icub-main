SET(KEYWORD "bml")

PROJECT(${KEYWORD})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(ODE REQUIRED)
FIND_PACKAGE(IPP REQUIRED)
#FIND_PACKAGE(CVBLOBS REQUIRED)



#search for includes
INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    #${CVBLOBS_INCLUDE_DIRS}
    ${OPENCV_INCLUDE_DIR}    
    ${ODE_INCLUDE_DIRS}	
    ${IPP_INCLUDE_DIRS}	

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
    ADD_LIBRARY(${KEYWORD}
    ${folder_source}
    ${folder_header}
    )
    #commented to be just a library 	
    #ADD_EXECUTABLE(${KEYWORD} 
    #${folder_source}
    #${folder_header}
    #)
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)



#INSTALL_TARGETS(/bin ${KEYWORD})
INSTALL_TARGETS(/lib ${KEYWORD})


