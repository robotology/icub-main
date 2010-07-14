PROJECT(iCubDemoY3)

SET(USE_NEW_CONFIG FALSE CACHE BOOL "Do you want to use a new experimental config method?")

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_EXECUTABLE(iCubDemoY3 ${folder_source} ${folder_header})

INSTALL(TARGETS iCubDemoY3 RUNTIME DESTINATION bin)

# organization of configuration files in installations
SET(appdir app/demoy3)
SET(d ${ICUB_DIR}/${appdir})

# configuration files
INSTALL(FILES ${d}/fullBody.txt ${d}/armRight1 ${d}/armLeft1 ${d}/rightLegRemap ${d}/leftLegRemap ${d}/headRemap ${d}/torsoRemap DESTINATION ${appdir})
INSTALL(FILES ${d}/configSim.ini DESTINATION ${appdir} RENAME config.ini)
