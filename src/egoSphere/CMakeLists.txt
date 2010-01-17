SET(KEYWORD "egoSphere")

PROJECT(${KEYWORD})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(KINEMATICS REQUIRED)
FIND_PACKAGE(GSL REQUIRED)
FIND_PACKAGE(SPMAP REQUIRED)
FIND_PACKAGE(SALIENCELIB REQUIRED) 

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${OPENCV_INCLUDE_DIR}
    ${KINEMATICS_INCLUDE_DIRS}
    ${GSL_INCLUDE_DIR}
    ${SPMAP_INCLUDE_DIRS}
    ${SALIENCELIB_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source 
	src/main.cpp
	src/AcousticMap.cpp
	src/EgoSphereInterfaces.cpp
	src/EgoSphereModule.cpp
	src/Framerate.cpp
	src/IModalityMap.cpp
	src/ObjectMap.cpp
	src/SphereProjector.cpp
	src/VisualMap.cpp
)
FILE(GLOB folder_header 
	include/iCub/AcousticMap.h
	include/iCub/EgoSphereInterfaces.h
	include/iCub/EgoSphereModule.h
	include/iCub/Framerate.h
	include/iCub/IModalityMap.h
	include/iCub/ObjectMap.h
	include/iCub/SphereProjector.h
	include/iCub/VisualMap.h
)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Source code for salience remote library
FILE(GLOB source_lib 
    src/RemoteEgoSphere.cpp
)
FILE(GLOB header_lib
    include/iCub/RemoteEgoSphere.h
    include/iCub/EgoSphereInterfaces.h
)
SOURCE_GROUP("Source Files" FILES ${source_lib})
SOURCE_GROUP("Header Files" FILES ${header_lib})


# Set up our main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)
TARGET_LINK_LIBRARIES(${KEYWORD}
    ${OPENCV_LIBRARIES}
    ${KINEMATICS_LIBRARIES}
    ${GSL_LIBRARIES}
    ${GSLCBLAS_LIBRARIES}
    ${SPMAP_LIBRARIES}
    ${SALIENCELIB_LIBRARIES}
)
INSTALL_TARGETS(/bin ${KEYWORD})

# Set up library
ADD_LIBRARY(egoSphereLib ${source_lib} ${header_lib})
TARGET_LINK_LIBRARIES(egoSphereLib
)
INSTALL_TARGETS(/lib egoSphereLib)

