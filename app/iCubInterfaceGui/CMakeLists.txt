set(appname iCubInterfaceGui)

file(GLOB icon_files ${CMAKE_CURRENT_SOURCE_DIR}/icons/*.png)

if (__ICUB_INSTALL_APPLICATIONS__)
icub_app(${appname})
icub_app_install(${appname} FILES ${icon_files} DESTINATION icons)
endif()

yarp_install(FILES ${icon_files} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname}/icons)


