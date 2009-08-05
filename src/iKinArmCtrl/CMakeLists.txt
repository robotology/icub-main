SET(PROJECTNAME iKinArmCtrl)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

FIND_PACKAGE(IKIN REQUIRED)
INCLUDE_DIRECTORIES(${IKIN_INCLUDE_DIRS})

IF(NOT IPOPT_FOUND)
   MESSAGE(FATAL_ERROR "IPOPT is strictly required!")
ENDIF(NOT IPOPT_FOUND)

SET(folder_header utils.h solver.h controller.h)
SET(folder_source utils.cpp solver.cpp controller.cpp main.cpp)

SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_header} ${folder_source})
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${IKIN_LIBRARIES})
INSTALL(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)




