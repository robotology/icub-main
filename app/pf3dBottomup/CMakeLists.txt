set(appname pf3dBottomup)

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB models ${CMAKE_CURRENT_SOURCE_DIR}/conf/models/*.bmp)

if (__ICUB_INSTALL_APPLICATIONS__)
icub_app(${appname})
icub_app_install(${appname} FILES ${conf})
icub_app_install(${appname} FILES ${models} DESTINATION models)
endif()

yarp_install(FILES ${conf} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${models} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname}/models)

