SET(PROJECTNAME iKinGazeCtrl)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

FIND_PACKAGE(IKIN REQUIRED)

FIND_PACKAGE(CTRLLIB REQUIRED)
INCLUDE_DIRECTORIES(${CTRLLIB_INCLUDE_DIRS})

IF(NOT IPOPT_FOUND)
   MESSAGE(FATAL_ERROR "IPOPT is strictly required!")
ENDIF(NOT IPOPT_FOUND)

INCLUDE_DIRECTORIES(${IKIN_INCLUDE_DIRS})

SET(folder_header gazeNlp.h utils.h solver.h controller.h localizer.h)
SET(folder_source gazeNlp.cpp utils.cpp solver.cpp controller.cpp localizer.cpp main.cpp)

SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_header} ${folder_source})
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${IKIN_LIBRARIES} ${CTRLLIB_LIBRARIES})
INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)




