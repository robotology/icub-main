SET(PROJECTNAME camCalib)

PROJECT(${PROJECTNAME})

SET(folder_source src/main.cpp
                  src/CamCalibModule.cpp
				  src/CalibToolFactory.cpp
				  src/PinholeCalibTool.cpp
				  src/SphericalCalibTool.cpp)
				  
SET(folder_header include/iCub/CamCalibModule.h
                   include/iCub/CalibToolFactory.h
				   include/iCub/ICalibTool.h
				   include/iCub/PinholeCalibTool.h
				   include/iCub/SphericalCalibTool.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${spMap_INCLUDE_DIRS}
                    ${OPENCV_INCLUDE_DIR}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} spMap
                                     ${OPENCV_LIBRARIES}
                                     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

