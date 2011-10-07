set(appname iCubInterfaceGui)

icub_app(${appname})


file(GLOB icon_files ${CMAKE_CURRENT_SOURCE_DIR}/icons/*.png)

icub_app_install(${appname}
  FILES ${icon_files}
  DESTINATION icons)





