CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(KEYWORD "saliencyBlobFinder")

PROJECT(${KEYWORD})

#FIND_PACKAGE(ICUB)
#FIND_PACKAGE(YARP)
#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_DIR}/conf ${YARP_DIR}/conf)
#FIND_PACKAGE(OpenCV REQUIRED)
#FIND_PACKAGE(IPP REQUIRED)



#search for includes
INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${ICUB_INCLUDE_DIRS}
    ${YARP_INCLUDE_DIRS}
    ${IPP_INCLUDE_DIRS}	 
)


# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})


# Set up our main executable
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )

    TARGET_LINK_LIBRARIES( ${KEYWORD} 
      			${YARP_LIBRARIES}
			${IPP_LIBRARIES}
    )

ELSE (folder_source)
    MESSAGE( "No source code files found. Please add something")
ENDIF (folder_source)


INSTALL_TARGETS(/bin ${KEYWORD})

