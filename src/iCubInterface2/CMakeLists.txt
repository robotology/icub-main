PROJECT(iCubInterface)

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
FIND_PACKAGE(CANLOADERLIB)

    
IF(USE_ICUB_MOD)
  ADD_DEFINITIONS(-DUSE_ICUB_MOD)
  FIND_PACKAGE(ICUBDEV) 
  FIND_PACKAGE(icubmod)
  IMPORT_DEVICES(${ICUB_DIR}/src/iCubInterface2/drivers.h icubmod)
ENDIF (USE_ICUB_MOD)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)

FILE(GLOB tmp ./canIdDiscoverer/*.h)
SET(folder_header ${folder_header} ${tmp})

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${CANLOADERLIB_INCLUDE_DIRS})
LINK_LIBRARIES(${CANLOADERLIB_LIBRARIES})


INCLUDE_DIRECTORIES(${ICUBDEV_INCLUDE_DIRS})

ADD_EXECUTABLE(iCubInterface ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(iCubInterface ${ICUBDEV_LIBRARIES})

INSTALL(TARGETS iCubInterface DESTINATION bin)





