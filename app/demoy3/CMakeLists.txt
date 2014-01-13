set(appname demoy3)

file(GLOB full_body      ${CMAKE_CURRENT_SOURCE_DIR}/fullBody.txt
                         ${CMAKE_CURRENT_SOURCE_DIR}/armRight1
                         ${CMAKE_CURRENT_SOURCE_DIR}/armLeft1
                         ${CMAKE_CURRENT_SOURCE_DIR}/rightLegRemap
                         ${CMAKE_CURRENT_SOURCE_DIR}/leftLegRemap
                         ${CMAKE_CURRENT_SOURCE_DIR}/torsoRemap
                         ${CMAKE_CURRENT_SOURCE_DIR}/headRemap)

file(GLOB hannover_txt   ${CMAKE_CURRENT_SOURCE_DIR}/hannover.txt)
file(GLOB hannover_files ${CMAKE_CURRENT_SOURCE_DIR}/hannover/armRightHannover
                         ${CMAKE_CURRENT_SOURCE_DIR}/hannover/armLeftHannover
                         ${CMAKE_CURRENT_SOURCE_DIR}/hannover/legRightHannover
                         ${CMAKE_CURRENT_SOURCE_DIR}/hannover/legLeftHannover
                         ${CMAKE_CURRENT_SOURCE_DIR}/hannover/torsoHannover
                         ${CMAKE_CURRENT_SOURCE_DIR}/hannover/headHannover)

file(GLOB crf_txt        ${CMAKE_CURRENT_SOURCE_DIR}/crf.txt)
file(GLOB crf_files      ${CMAKE_CURRENT_SOURCE_DIR}/crf/armRight
                         ${CMAKE_CURRENT_SOURCE_DIR}/crf/armLeft
                         ${CMAKE_CURRENT_SOURCE_DIR}/crf/legRight
                         ${CMAKE_CURRENT_SOURCE_DIR}/crf/legLeft
                         ${CMAKE_CURRENT_SOURCE_DIR}/crf/torso
                         ${CMAKE_CURRENT_SOURCE_DIR}/crf/head)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml.template)

yarp_install(FILES ${full_body} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${hannover_txt} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${hannover_files} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname}/hannover)
yarp_install(FILES ${crf_txt} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${crf_files} DESTINATION ${ICUB_CONTEXTS_INSTALL_DIR}/${appname}/crf)
yarp_install(FILES ${scripts} DESTINATION ${ICUB_APPLICATIONS_TEMPLATES_INSTALL_DIR})


