set(appname robotScripting)

icub_app(${appname})

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml.template)
file(GLOB example ${CMAKE_CURRENT_SOURCE_DIR}/example/*)

icub_app_install(${appname} 
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/scripting.py)	

icub_app_install(${appname}
  FILES ${scripts}
  DESTINATION scripts)

icub_app_install(${appname}
  FILES ${example}
  DESTINATION example)
