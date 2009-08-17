SET(APP_NAME iCubGenova01)

PROJECT($APP_NAME)

SET(appdir app/${APP_NAME})
SET(scripts "${ICUB_DIR}/${appdir}/scripts")
SET(conf "${ICUB_DIR}/${appdir}/conf")
SET(dapp "${ICUB_APP}/default")

INSTALL(CODE "MESSAGE(\"Installing ${APP_NAME}\")")

# some scripts
INSTALL(FILES 
  ${scripts}/config.sh
  ${scripts}/config-nodes.sh
  DESTINATION ${dapp}/scripts
  PERMISSIONS 
  OWNER_EXECUTE OWNER_READ OWNER_WRITE 
  GROUP_READ GROUP_WRITE
  WORLD_READ)

# configuration files
INSTALL(DIRECTORY
  ${conf}
  DESTINATION ${dapp}
  PATTERN 
  ".svn" EXCLUDE
  PERMISSIONS OWNER_READ OWNER_WRITE 
  GROUP_READ GROUP_WRITE
  WORLD_READ)

#INSTALL(FILES ${d}/config.sh DESTINATION ${appdir} RENAME config.ini)
#INSTALL(FILES ${d}/config.sh DESTINATION ${dest})
