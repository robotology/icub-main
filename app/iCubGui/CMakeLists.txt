set(appname iCubGui)

icub_app(${appname})

file(GLOB covers ${CMAKE_CURRENT_SOURCE_DIR}/covers/*.obj)
file(GLOB files ${CMAKE_CURRENT_SOURCE_DIR}/*.ini)
file(GLOB icon_files ${CMAKE_CURRENT_SOURCE_DIR}/icons/*.png)

icub_app_install(${appname}
  FILES ${covers}
  DESTINATION covers)

icub_app_install(${appname}
  FILES ${icon_files}
  DESTINATION icons)

icub_app_install(${appname}
  FILES ${files})

icub_add_target(${appname})



