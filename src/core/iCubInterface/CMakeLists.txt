PROJECT(iCubInterface)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)

FILE(GLOB tmp ./canIdDiscoverer/*.h)
SET(folder_header ${folder_header} ${tmp})

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../tools/canLoader/canLoaderLib
                    ${iCubDev_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

if(ICUB_USE_INTERFACEGUI)
	add_definitions(-D_USE_INTERFACEGUI)
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../../tools/iCubInterfaceGui/iCubInterfaceGuiLib)
endif(ICUB_USE_INTERFACEGUI)

ADD_EXECUTABLE(iCubInterface ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(iCubInterface iCubDev 
                                    ${YARP_LIBRARIES}
                                    canLoaderLib 
                                    icubmod)

if(ICUB_USE_INTERFACEGUI)
	TARGET_LINK_LIBRARIES(iCubInterface iCubInterfaceGuiLib)
endif(ICUB_USE_INTERFACEGUI)

INSTALL(TARGETS iCubInterface DESTINATION bin)

