SET(KEYWORD "visualAttentionB09")


PROJECT(${KEYWORD})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(ODE REQUIRED)
FIND_PACKAGE(IPP REQUIRED)
#FIND_PACKAGE(QT)

#---------------------------------------
FIND_PACKAGE(GtkPlus REQUIRED)
ADD_DEFINITIONS(${GTKPLUS_C_FLAGS})
LINK_LIBRARIES(${GTKPLUS_LINK_FLAGS})
INCLUDE_DIRECTORIES(${GTKPLUS_INCLUDE_DIR})
#---------------------------------------

#--------------------------
#FIND_PACKAGE(LOGPOLAR REQUIRED)
#LINK_LIBRARIES(${LOGPOLAR_LIBRARIES})
#INCLUDE_DIRECTORIES(${LOGPOLAR_INCLUDE_DIRS})
#--------------------------


#search for includes
INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${OPENCV_INCLUDE_DIR}    
    ${ODE_INCLUDE_DIRS}	
    ${IPP_INCLUDE_DIRS}	 
    #/opt/intel/ipp/6.1.1.042/em64t/include
    #---linux    
    #$ENV{QT_INCLUDE_DIR}/include
    #$ENV{QT_INCLUDE_DIR}/include/Qt
    #$ENV{QT_INCLUDE_DIR}/include/Qt3Support 
    #--windows
    $ENV{QT_INCLUDE_DIR}
)

IF(NOT IPP_INCLUDE_DIRS)
	MESSAGE(ERROR ": No IPP_INCLUDE_DIR found. Please check it out")	
ENDIF(NOT IPP_INCLUDE_DIRS)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

SET(ImageProcessModuleSRC
    src/ImageProcessModule.cpp
)


LINK_DIRECTORIES(
    ${ICUB_SOURCE_DIR}
    #/opt/intel/ipp/sharedlib	
)

#LINK_LIBRARIES(debug "$ENV{ICUB_ROOT}/lib/debug/cvBlobs.lib" 
#	optimized "$ENV{ICUB_ROOT}/lib/debug/cvBlobs.lib")

LINK_LIBRARIES(
    ${OPENCV_LIBRARIES}
    ${ODE_LIBRARIES}
    ${IDL_LIBRARIES}
    ${IPP_LIBRARIES} 
    #ippi	
)

# Set up our main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )	
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)


ADD_LIBRARY(ImageProcessModule
    ${ImageProcessModuleSRC}
    ${folder_header}
)

INSTALL_TARGETS(/lib ${KEYWORD})
INSTALL_TARGETS(/bin ${KEYWORD})

