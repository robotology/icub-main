SET(PROJECTNAME vergence)
PROJECT(${PROJECTNAME})

FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
		            ${iKin_INCLUDE_DIRS}
		            ${logPolar_INCLUDE_DIRS}			
		    )

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} 
                     iKin
                     ${YARP_LIBRARIES}
     				 logPolar
				     ${ICUB_LIBRARIES}
    				 )

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

