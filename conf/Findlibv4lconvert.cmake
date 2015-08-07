#
# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author:  Alberto Cardellino
# email:   alberto.cardellino@iit.it
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#

# Try to find the libv4lconvert library.
# Once done this will define the following variables:
#
# libv4lconvert_FOUND         - System has libv4lconvert
# libv4lconvert_INCLUDE_DIRS  - libv4lconvert include directory
# libv4lconvert_LIBRARIES     - libv4lconvert libraries
# libv4lconvert_DEFINITIONS   - Additional compiler flags for libv4lconvert
# libv4lconvert_VERSION       - libv4lconvert version
# libv4lconvert_MAJOR_VERSION - libv4lconvert major version
# libv4lconvert_MINOR_VERSION - libv4lconvert minor version
# libv4lconvert_PATCH_VERSION - libv4lconvert patch version
# libv4lconvert_TWEAK_VERSION - libv4lconvert tweak version

include(MacroStandardFindModule)
macro_standard_find_module(libv4lconvert libv4lconvert)

# Set package properties if FeatureSummary was included
if(COMMAND set_package_properties)
    set_package_properties(libv4lconvert PROPERTIES DESCRIPTION "Video4linux frame format conversion library")
endif()
