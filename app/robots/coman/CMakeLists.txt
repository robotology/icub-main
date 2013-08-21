set(appname coman)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)
file(GLOB conf    ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.xml ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.yaml)
file(GLOB conf_hw ${CMAKE_CURRENT_SOURCE_DIR}/conf/hardware/*.xml)
file(GLOB conf_wr ${CMAKE_CURRENT_SOURCE_DIR}/conf/wrappers/*.xml)

if (__ICUB_INSTALL_APPLICATIONS__)
icub_app(${appname})
icub_app_install(${appname} 
                           FILES ${scripts}
                           DESTINATION scripts)

icub_app_install(${appname} 
                           FILES ${conf})                       
    
icub_app_install(${appname}
                                FILES ${conf_hw}
                                DESTINATION conf/hardware)                       
    
icub_app_install(${appname} 
                                FILES ${conf_wr}
                                DESTINATION conf/wrappers)                       
endif()

yarp_install(FILES ${scripts} DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname}/scripts)
yarp_install(FILES ${conf} DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${conf_hw} DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname}/hardware)
yarp_install(FILES ${conf_wr} DESTINATION ${ICUB_ROBOTS_INSTALL_DIR}/${appname}/wrappers)
