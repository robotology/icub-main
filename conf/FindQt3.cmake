#message(Find Qt3 form iCub package)

# save current CMAKE_MODULE_PATH, disable it 
# to avoid recursive calls to FindGLUT
set(_CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH "")

set(Qt3_DIR $ENV{Qt3_DIR})

if (Qt3_DIR)
	message(${Qt3_DIR})
endif(Qt3_DIR)
	
find_package(Qt3)

if (QT_FOUND)
	set(Qt3_INCLUDE_DIRS ${QT_INCLUDE_DIR})
	set(Qt3_LIBRARIES ${QT_LIBRARIES})
	set(Qt3_FOUND TRUE)
endif(QT_FOUND)

# push back original CMAKE_MODULE_PATH
set(CMAKE_MODULE_PATH ${_CMAKE_MODULE_PATH})