SET(KEYWORD "soundLocalization")
SET(DUMMY "soundLocalizationDummy")

PROJECT(${KEYWORD})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${ICUB_DIR}/src/detectSound  # seem to need YARPFft.h
)

# seem to need YARPFft.cpp -- ideally would use a library
SET(external_files ${ICUB_DIR}/src/detectSound/YARPFft.cpp)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

LINK_LIBRARIES(
)
# THIS IS COMMENTED BECAUSE OF A LINKING ERROR ON THE BLADES 27.10.2008, DO NOT COMMIT THIS.
# Set up our main executable.
#IF (folder_source)
#    ADD_EXECUTABLE(${KEYWORD} 
#        ${folder_source} 
#        ${folder_header}
#	    ${external_files}
#    )
#ELSE (folder_source)
#    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
#ENDIF (folder_source)

#INSTALL_TARGETS(/bin ${KEYWORD})


PROJECT(${DUMMY})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/test/include
)

# Search for source code.
FILE(GLOB folder_source_dummy test/src/*.cpp test/src/*.cc test/src/*.c)
FILE(GLOB folder_header_dummy test/include/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source_dummy})
SOURCE_GROUP("Header Files" FILES ${folder_header_dummy})

LINK_LIBRARIES(
)

# Set up our main executable.
IF (folder_source_dummy)
    ADD_EXECUTABLE(${DUMMY} 
        ${folder_source_dummy} 
        ${folder_header_dummy}
	    ${external_files_dummy}
    )
ELSE (folder_source_dummy)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source_dummy)

INSTALL_TARGETS(/bin ${DUMMY})
