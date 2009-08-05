PROJECT(canLoaderServer)

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP)

SET(folder_source main.cpp driver.cpp downloader.cpp)
SET(folder_header driver.h downloader.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

IF (WIN32 AND NOT CYGWIN)
    FIND_LIBRARY(libntcan ntcan ${ICUB_DIR}/src/modules/esdCanAPi/winnt/lib/)
    INCLUDE_DIRECTORIES(${ICUB_DIR}/src/modules/esdCanAPi/winnt/include)
ELSE (WIN32 AND NOT CYGWIN)
    FIND_LIBRARY(libntcan ntcan ${ICUB_DIR}/src/modules/esdCanAPi/linux/lib/)
    INCLUDE_DIRECTORIES(${ICUB_DIR}/src/modules/esdCanAPi/linux/include)
ENDIF (WIN32 AND NOT CYGWIN)

LINK_LIBRARIES(${libntcan})

ADD_EXECUTABLE(canLoaderServer ${folder_source} ${folder_header})

INSTALL_TARGETS(/bin canLoaderServer) 
