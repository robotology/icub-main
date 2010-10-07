set(appname demoy3)

icub_app(${appname})

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml.template)
file(GLOB files ${CMAKE_CURRENT_SOURCE_DIR}/*)

icub_app_install(${appname}
  FILES ${scripts}
  DESTINATION scripts)

icub_app_install(${appname}
  FILES ${files})

icub_add_target(${appname})



