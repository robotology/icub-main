set(appname iCubInterfaceGui)

file(GLOB icon_files ${CMAKE_CURRENT_SOURCE_DIR}/icons/*.png)

yarp_install(FILES ${icon_files} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname}/icons)


