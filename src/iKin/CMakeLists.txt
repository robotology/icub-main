SET(PROJECTNAME iKin)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

FIND_PACKAGE(CTRLLIB REQUIRED)
INCLUDE_DIRECTORIES(${CTRLLIB_INCLUDE_DIRS})

SET(ADD_IPOPT TRUE CACHE BOOL "Dou you want to add IPOPT library?")
IF(ADD_IPOPT)
   FIND_PACKAGE(IPOPT)
   IF(NOT IPOPT_FOUND)
      MESSAGE(STATUS "IPOPT not found! Keep on building...")
   ENDIF(NOT IPOPT_FOUND)
ENDIF(ADD_IPOPT)

SET(folder_source src/iKinFwd.cpp src/iKinInv.cpp src/iKinHlp.cpp)
SET(folder_header include/iCub/iKinFwd.h include/iCub/iKinInv.h
                  include/iCub/iKinVocabs.h include/iCub/iKinHlp.h)

IF(IPOPT_FOUND)
   SET(folder_source ${folder_source} src/iKinIpOpt.cpp src/iKinSlv.cpp)
   SET(folder_header ${folder_header} include/iCub/iKinIpOpt.h include/iCub/iKinSlv.h)
ENDIF(IPOPT_FOUND)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include ${IPOPT_INC_DIRS})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})



