PREPARE_DEVICE(icubarmcalibrator TYPE iCubArmCalibrator INCLUDE iCubArmCalibrator.h)
PREPARE_DEVICE(icubarmcalibratorj8 TYPE iCubArmCalibratorJ8 INCLUDE iCubArmCalibratorJ8.h)

if (NOT SKIP_armcalibrator)
  include_directories(${YARP_INCLUDE_DIRS})

  add_library(armcalibrator iCubArmCalibrator.cpp iCubArmCalibrator.h)

  target_link_libraries(armcalibrator ${YARP_LIBRARIES})
  icub_export_library(armcalibrator "" include/iCub "")
endif (NOT SKIP_armcalibrator)

if (NOT SKIP_armcalibratorj8)
  include_directories(${YARP_INCLUDE_DIRS})

  add_library(armcalibratorj8 iCubArmCalibratorJ8.cpp iCubArmCalibratorJ8.h)

  target_link_libraries(armcalibratorj8 ${YARP_LIBRARIES})
  icub_export_library(armcalibratorj8 "" include/iCub "")

endif (NOT SKIP_armcalibratorj8)

