SET(PROJECTNAME iKin)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
FIND_PACKAGE(CTRLLIB REQUIRED)

FIND_PACKAGE(IPOPT)
IF(NOT IPOPT_FOUND)
   MESSAGE(STATUS "IPOPT not found! Keep on building ...")
ENDIF(NOT IPOPT_FOUND)

SET(ADD_IPOPT ${IPOPT_FOUND} CACHE BOOL "Dou you want to add IPOPT library?")
IF(NOT ADD_IPOPT)
   SET(IPOPT_INC_DIRS "")
   SET(IPOPT_LIB "")
   SET(IPOPT_LIB_R "")
   SET(IPOPT_LIB_D "")
ENDIF(NOT ADD_IPOPT)

SET(folder_source src/iKinFwd.cpp src/iKinInv.cpp src/iKinHlp.cpp)
SET(folder_header include/iCub/iKinFwd.h include/iCub/iKinInv.h
                  include/iCub/iKinVocabs.h include/iCub/iKinHlp.h)
				  
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include ${CTRLLIB_INCLUDE_DIRS})

IF(IPOPT_FOUND AND ADD_IPOPT)
   SET(folder_source ${folder_source} src/iKinIpOpt.cpp src/iKinSlv.cpp)
   SET(folder_header ${folder_header} include/iCub/iKinIpOpt.h include/iCub/iKinSlv.h)
   INCLUDE_DIRECTORIES(${IPOPT_INC_DIRS})
ENDIF(IPOPT_FOUND AND ADD_IPOPT)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${CTRLLIB_LIBRARIES})

IF(MSVC)
   SET_TARGET_PROPERTIES(${PROJECTNAME} PROPERTIES COMPILE_FLAGS /wd4996)
ENDIF(MSVC)

IF(WIN32 AND NOT CYGWIN)
   SET_TARGET_PROPERTIES(${PROJECTNAME} PROPERTIES DEBUG_POSTFIX "d")
ENDIF(WIN32 AND NOT CYGWIN)

INSTALL(TARGETS ${PROJECTNAME} DESTINATION lib)
INSTALL(FILES ${folder_header} DESTINATION include/iCub)

