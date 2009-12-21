SET(KEYWORD "colourProcessor")

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
    #${ODE_INCLUDE_DIRS}	
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


#LINK_LIBRARIES(
#    ${OPENCV_LIBRARIES}
#    ${IPP_LIBRARIES} 
#    #${CVBLOBS_LIBRARIES}
#    #${ODE_LIBRARIES}
#    #${IDL_LIBRARIES}	 
#)

# Set up our main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )
    IF(WIN32)	
    TARGET_LINK_LIBRARIES( ${KEYWORD} 
      ${OPENCV_LIBRARIES}
      ${IPP_LIBRARIES}
    )
    ELSE(WIN32) #required in linux
     TARGET_LINK_LIBRARIES( ${KEYWORD}
       ${OPENCV_LIBRARIES}
       ${IPP_LIBRARIES} 	
       pthread
     )
    ENDIF (WIN32)	
ELSE (folder_source)
    MESSAGE( "No source code files found. Please add something")
ENDIF (folder_source)


INSTALL_TARGETS(/bin ${KEYWORD})

