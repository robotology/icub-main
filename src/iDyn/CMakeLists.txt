SET(PROJECTNAME iDyn)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
FIND_PACKAGE(IKIN REQUIRED)

INCLUDE_DIRECTORIES(${IKIN_INCLUDE_DIRS})

SET(folder_source src/iDyn.cpp
                  src/iDynInv.cpp
                  src/iDynFwd.cpp
                  src/iFC.cpp)

SET(folder_header include/iCub/iDyn.h
                  include/iCub/iDynInv.h
                  include/iCub/iDynFwd.h
                  include/iCub/iFC.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${IKIN_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION lib)
INSTALL(FILES ${folder_header} DESTINATION include/iCub)



