SET(appdir ${CMAKE_CURRENT_SOURCE_DIR})
SET(dapp ${ICUB_APP})

INSTALL(CODE "MESSAGE(\"Parsing ${appdir}\")")

INSTALL(DIRECTORY
  ${appdir}
  DESTINATION ${dapp}
  USE_SOURCE_PERMISSIONS
  PATTERN "CVS" EXCLUDE
  PATTERN
  "CMakeLists.txt" EXCLUDE
  PATTERN
  "CMakeLists.txt~" EXCLUDE
  PATTERN
  "CMakeFiles" EXCLUDE
  PATTERN
  "Makefile" EXCLUDE
  PATTERN
  "cmake_install.cmake" EXCLUDE)


