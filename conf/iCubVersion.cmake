# Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# Jan 2011. It is time to add versioning information to the iCub builds.

set(ICUB_VERSION_MAJOR "1")
set(ICUB_VERSION_MINOR "1")
set(ICUB_VERSION_PATCH "15")
set(ICUB_VERSION_MODIFIER "")
set(ICUB_VERSION_ABI "0")

set(CPACK_PACKAGE_VERSION_MAJOR "${ICUB_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${ICUB_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${ICUB_VERSION_PATCH}${ICUB_VERSION_MODIFIER}")

set(ICUB_GENERIC_VERSION "${ICUB_VERSION_MAJOR}.${ICUB_VERSION_MINOR}.${ICUB_VERSION_PATCH}")
set(ICUB_GENERIC_SOVERSION "${ICUB_VERSION_ABI}")
