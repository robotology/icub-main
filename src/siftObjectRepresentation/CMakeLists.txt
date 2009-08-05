# The name of our project is "SIFT".  CMakeLists files in this project can
# refer to the root source directory of the project as ${SIFT_SOURCE_DIR} and
# to the root binary directory of the project as ${SIFT_BINARY_DIR}.
SET(KEYWORD "SiftCV")

PROJECT (${KEYWORD})


# If you need YARP, make sure you have the YARP_DIR environment variable set, and then use:
#SET(CMAKE_MODULE_PATH /home/dario/cmake-2.4.7/Modules)
#SET(YARP_DIR /home/dario/yarp-2.1.7/)
# SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is yarp?")
# SET(ICUB_DIR "$ENV{ICUB_ROOT}" CACHE LOCATION "where is icub?")
# SET(CMAKE_MODULE_PATH "$ENV{YARP_ROOT}/conf")

#INCLUDE(YarpProgram)

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(KINEMATICS REQUIRED) # kinematics
FIND_PACKAGE(GtkPlus REQUIRED) #This needs FIND_PACKAGE(ICUB)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(GSL REQUIRED)

INCLUDE_DIRECTORIES(
	${OPENCV_INCLUDE_DIR}
	${YARP_INCLUDE_DIR}
        ${KINEMATICS_INCLUDE_DIRS}
	${GTKPLUS_INCLUDE_DIR}
	${GSL_INCLUDE_DIR}
	${PROJECT_SOURCE_DIR}/include
)


FILE(GLOB source_files src/*.cpp src/*.cc src/*.c)
FILE(GLOB header_files include/*.h include/*.hpp)
SOURCE_GROUP("Source Files" FILES ${source_files})
SOURCE_GROUP("Header Files" FILES ${header_files})

ADD_EXECUTABLE(siftObjectRepresentation
    main.cpp
    ${source_files} 
    ${header_files}
)

#Make sure the linker can find the Hello library once it is built.
LINK_DIRECTORIES (
    ${PROJECT_BINARY_DIR}/include
    ${PORJECT_BINARY_DIR}/src
)

TARGET_LINK_LIBRARIES (
    siftObjectRepresentation  
    ${OPENCV_LIBRARIES} 
    ${YARP_LIBRARIES} 
    ${GSL_LIBRARIES}     
    ${KINEMATICS_LIBRARIES}
)


SET(CMAKE_BUILD_TYPE Debug)

#ADD_DEFINITIONS(-march=pentium4 -mfpmath=sse -msse3)
ADD_DEFINITIONS(${GTKPLUS_C_FLAGS})

#to use "gprof"
# ADD_DEFINITIONS(-pg)
# ADD_DEFINITIONS(-O0 -g)
