set(appname demoy3)

icub_app(${appname})

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml.template)
file(GLOB positions ${CMAKE_CURRENT_SOURCE_DIR}/*)
file(GLOB crf-files ${CMAKE_CURRENT_SOURCE_DIR}/crf/*)

icub_app_install(${appname}
  FILES ${scripts}
  DESTINATION scripts)

icub_app_install(${appname}
  FILES ${crf-files}
  DESTINATION crf)

icub_app_install(${appname}
  FILES ${positions})




