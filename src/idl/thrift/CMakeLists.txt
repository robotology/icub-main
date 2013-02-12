# Copyright: (C) 2012 Istituto Italiano di Tecnologia (IIT)
# Authors: Elena Ceseracciu
# CopyPolicy: Released under the terms of the GNU GPL v2.0 and later.

#SET(THRIFT_FILES_LIST iCub/actionsRendering/actionsRenderingEngineInterface.thrift iCub/actionsRendering/gazeHead.thrift)
SET(THRIFT_FILES_LIST iCub/velocityControl/VelocityControlInterface.thrift 
					  iCub/iSpeak/iSpeakThrift.thrift)
include(${YARP_MODULE_PATH}/YarpIDL.cmake)

set(generated_libs_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../libraries/interfaces/thriftGenerated")
foreach(thrift_file ${THRIFT_FILES_LIST})
   yarp_idl_to_dir(${thrift_file} ${generated_libs_dir})
endforeach()


