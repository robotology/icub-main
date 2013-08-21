set(appname pf3dTracker)

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini
                ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.txt
                           ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.csv)

file(GLOB models ${CMAKE_CURRENT_SOURCE_DIR}/conf/models/*.bmp
                             ${CMAKE_CURRENT_SOURCE_DIR}/conf/models/*.csv)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

if (__ICUB_INSTALL_APPLICATIONS__)
icub_app(${appname})
icub_app_install(${appname} FILES ${scripts} DESTINATION scripts)
icub_app_install(${appname} FILES ${conf})
icub_app_install(${appname} FILES ${models} DESTINATION models)
endif()

yarp_install(FILES ${conf} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${models} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname}/models)
yarp_install(FILES ${scripts} DESTINATION ${ICUB_APPLICATIONS_TEMPLATES_INSTALL_DIR})

