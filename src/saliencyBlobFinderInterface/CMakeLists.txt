SET(KEYWORD "saliencyBlobFinderInterface")

PROJECT(${KEYWORD})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_DIR}/conf ${ICUB_DIR}/conf)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(IPP REQUIRED)

#---------------------------------------
FIND_PACKAGE(GtkPlus REQUIRED)
ADD_DEFINITIONS(${GTKPLUS_C_FLAGS})
LINK_LIBRARIES(${GTKPLUS_LINK_FLAGS})
INCLUDE_DIRECTORIES(${GTKPLUS_INCLUDE_DIR})
#---------------------------------------


#search for includes
INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS}
    ${ICUB_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}    	
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
    /opt/intel/ipp/sharedlib	
)




# Set up our main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )	
	IF(WIN32)
	TARGET_LINK_LIBRARIES( ${KEYWORD} 
    ${OpenCV_LIBRARIES}
    ${YARP_LIBRARIES}
    ${IDL_LIBRARIES}
    ${IPP_LIBRARIES}	 
   )
	ELSE(WIN32)
     TARGET_LINK_LIBRARIES( ${KEYWORD} 	
    ${OpenCV_LIBRARIES}
    ${YARP_LIBRARIES}
    ${IDL_LIBRARIES}
    ${IPP_LIBRARIES}	
	 pthread 
   )
	ENDIF(WIN32)
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)


INSTALL_TARGETS(/bin ${KEYWORD})

