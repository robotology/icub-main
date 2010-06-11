SET(PROJECTNAME iDyn)

PROJECT(${PROJECTNAME})

SET(folder_source src/iDyn.cpp
                  src/iDynInv.cpp
                  src/iDynFwd.cpp
		  src/iDynBody.cpp
                  src/iDynTransform.cpp)

SET(folder_header include/iCub/iDyn/iDyn.h
                  include/iCub/iDyn/iDynInv.h
                  include/iCub/iDyn/iDynFwd.h
		  include/iCub/iDyn/iDynBody.h
                  include/iCub/iDyn/iDynTransform.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${iKin_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} iKin
                                     ${YARP_LIBRARIES})
				   

icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DESTINATION include/iCub/iDyn
                                   DEPENDS iKin
                                   FILES ${folder_header})
