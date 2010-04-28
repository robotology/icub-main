SET(KEYWORD "vergenceController")

PROJECT(${KEYWORD})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)

#------------------------------
#---------IPP LIBRARY--------
#FIND_PACKAGE(IPP REQUIRED)
#INCLUDE_DIRECTORIES(${IPP_INCLUDE_DIRS})
#LINK_LIBRARIES(${IPP_LIBRARIES})
#------------------------------


#search for includes
INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
)


# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})


LINK_DIRECTORIES(
    ${ICUB_SOURCE_DIR}
)



LINK_LIBRARIES(
    ${IDL_LIBRARIES}	 
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


