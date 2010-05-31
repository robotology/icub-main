SET(PROJECTNAME iKin)

PROJECT(${PROJECTNAME})

IF(NOT ICUB_USE_IPOPT)
   MESSAGE(STATUS "IPOPT not found! Keep on building ...")
   UNSET(IPOPT_INCLUDE_DIRS)
   UNSET(IPOPT_LIBRARIES)
   UNSET(IPOPT_LIBRARIES_RELEASE)
   UNSET(IPOPT_LIBRARIES_DEBUG)
ENDIF(NOT ICUB_USE_IPOPT)

#SET(ADD_IPOPT ${IPOPT_FOUND} CACHE BOOL "Dou you want to add IPOPT library?")
#IF(NOT ADD_IPOPT)
#   UNSET(IPOPT_INCLUDE_DIRS)     
#   UNSET(IPOPT_LIBRARIES)        
#   UNSET(IPOPT_LIBRARIES_RELEASE)
#   UNSET(IPOPT_LIBRARIES_DEBUG)  
#ENDIF(NOT ADD_IPOPT)

SET(folder_source src/iKinFwd.cpp
                  src/iKinInv.cpp
                  src/iKinHlp.cpp)

SET(folder_header include/iCub/iKin/iKinFwd.h
                  include/iCub/iKin/iKinInv.h
                  include/iCub/iKin/iKinVocabs.h
                  include/iCub/iKin/iKinHlp.h)
				  
IF(ICUB_USE_IPOPT)
   SET(folder_source ${folder_source}
                     src/iKinIpOpt.cpp
                     src/iKinSlv.cpp)

   SET(folder_header ${folder_header}
                     include/iCub/iKin/iKinIpOpt.h
                     include/iCub/iKin/iKinSlv.h)

   INCLUDE_DIRECTORIES(${IPOPT_INCLUDE_DIRS})
ENDIF(ICUB_USE_IPOPT)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include ${YARP_INCLUDE_DIRS} ${ctrlLib_INCLUDE_DIRS})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ctrlLib)

IF(ICUB_USE_IPOPT)
   TARGET_LINK_LIBRARIES(${PROJECTNAME} ${IPOPT_LIBRARIES})
ENDIF(ICUB_USE_IPOPT)

# hopefully no longer needed
#IF(MSVC)
#   SET_TARGET_PROPERTIES(${PROJECTNAME} PROPERTIES COMPILE_FLAGS /wd4996)
#ENDIF(MSVC)

IF(WIN32 AND NOT CYGWIN)
   SET_TARGET_PROPERTIES(${PROJECTNAME} PROPERTIES DEBUG_POSTFIX "d")
ENDIF(WIN32 AND NOT CYGWIN)

icub_export_library(${PROJECTNAME}
                   "${PROJECT_SOURCE_DIR}/include;${ctrlLib_INCLUDE_DIRS}"
                   include/iCub/iKin ${folder_header})


