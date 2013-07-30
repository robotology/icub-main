set(appname demoy3)

icub_app(${appname})

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml.template)
file(GLOB positions ${CMAKE_CURRENT_SOURCE_DIR}/*)
file(GLOB crf-files ${CMAKE_CURRENT_SOURCE_DIR}/crf/*)
file(GLOB hannover-files ${CMAKE_CURRENT_SOURCE_DIR}/hannover/*)

icub_app_install(${appname}
  FILES ${scripts}
  DESTINATION scripts)

icub_app_install(${appname}
  FILES ${crf-files}
  DESTINATION crf)

icub_app_install(${appname}
  FILES ${hannover-files}
  DESTINATION hannover)

icub_app_install(${appname}
  FILES ${positions})

#yarp_install(FILES ${crf_files} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname}/crf)
#yarp_install(FILES ${hannover_files} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname}/hannover)
#yarp_install(FILES ${positions} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname}/positions)
yarp_install(FILES ${scripts} DESTINATION ${ICUB_APPLICATIONS_TEMPLATES_INSTALL_DIR})


