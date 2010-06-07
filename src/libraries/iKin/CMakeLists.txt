SET(PROJECTNAME iKin)

PROJECT(${PROJECTNAME})

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
ENDIF(ICUB_USE_IPOPT)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${ctrlLib_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

IF(ICUB_USE_IPOPT)
   INCLUDE_DIRECTORIES(${IPOPT_INCLUDE_DIRS})
ENDIF(ICUB_USE_IPOPT)

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ctrlLib
                                     ${YARP_LIBRARIES})

IF(ICUB_USE_IPOPT)
   TARGET_LINK_LIBRARIES(${PROJECTNAME} ${IPOPT_LIBRARIES})
ENDIF(ICUB_USE_IPOPT)

IF(ICUB_USE_IPOPT)
if (NOT ICUB_EXPERIMENTAL)
   ICUB_EXPORT_LIBRARY(${PROJECTNAME}
                      "${PROJECT_SOURCE_DIR}/include;${ctrlLib_INCLUDE_DIRS};${IPOPT_INCLUDE_DIRS}"
                      include/iCub/iKin ${folder_header})
else()
    icub_export_library2(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                    EXTERNAL_INCLUDE_DIRS ${IPOPT_INCLUDE_DIRS}
                                    DESTINATION include/iCub/iKin
                                    DEPENDS ctrlLib
                                    FILES ${folder_header})
endif()
                                    
ELSE(ICUB_USE_IPOPT)
if (NOT ICUB_EXPERIMENTAL)
    ICUB_EXPORT_LIBRARY(${PROJECTNAME}
                   "${PROJECT_SOURCE_DIR}/include;${ctrlLib_INCLUDE_DIRS}"
                      include/iCub/iKin ${folder_header})
else()
    icub_export_library2(${PROJECTNAME} INTERNAL_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include"
                                    DEPENDS ctrlLib
                                    DESTINATION include/iCub/iKin
                                    FILES ${folder_header})
endif()
                                    
ENDIF(ICUB_USE_IPOPT)


