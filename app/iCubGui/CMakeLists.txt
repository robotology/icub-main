set(appname iCubGui)

icub_app(${appname})

file(GLOB convers ${CMAKE_CURRENT_SOURCE_DIR}/covers/*.obj)
file(GLOB files ${CMAKE_CURRENT_SOURCE_DIR}/*.ini)
file(GLOB icons ${CMAKE_CURRENT_SOURCE_DIR}/icons/*.png)

icub_app_install(${appname}
  FILES ${convers}
  DESTINATION convers)

icub_app_install(${appname}
  FILES ${icons}
  DESTINATION icons)

icub_app_install(${appname}
  FILES ${files})

icub_add_target(${appname})



