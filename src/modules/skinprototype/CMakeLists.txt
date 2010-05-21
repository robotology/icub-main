

IF (COMPILE_DEVICE_LIBRARY)
  PREPARE_DEVICE(skinprototype TYPE SkinPrototype INCLUDE SkinPrototype.h
				WRAPPER grabber)
ENDIF (COMPILE_DEVICE_LIBRARY)

IF (NOT SKIP_static_grabber)
FIND_PACKAGE(YARP REQUIRED)

ADD_LIBRARY(skinprototype SkinPrototype.h SkinPrototype.cpp)
ENDIF (NOT SKIP_static_grabber)