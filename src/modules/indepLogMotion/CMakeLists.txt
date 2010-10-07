SET(KEYWORD "indepLogMotion")
PROJECT(${KEYWORD})

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${logpolar_INCLUDE_DIRS}
    ${YARP_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_EXECUTABLE(${KEYWORD} 
               ${folder_source} 
               ${folder_header})

TARGET_LINK_LIBRARIES(${KEYWORD}
                      ${YARP_LIBRARIES})	

INSTALL_TARGETS(/bin ${KEYWORD})
