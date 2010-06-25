CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECTNAME vergence)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_DIR}/conf ${YARP_DIR}/conf)
FIND_PACKAGE(IPP REQUIRED)

FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
		    ${iKin_INCLUDE_DIRS}
		    ${logPolar_INCLUDE_DIRS}
		    ${IPP_INCLUDE_DIRS}
		    )

MESSAGE("logPolar_INCLUDE_DIRS  ${logPolar_INCLUDE_DIRS} logPolar_LIBRARIES ${ICUB_LIBRARIES}  ")

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${iKin_LIBRARIES}
                                     ${YARP_LIBRARIES}
     				     ${logPolar_LIBRARIES}
				     ${ICUB_LIBRARIES}
				     ${IPP_LIBRARIES}
									 )

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

