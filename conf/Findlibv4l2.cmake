#
# Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Author:  Alberto Cardellino
# email:   alberto.cardellino@iit.it
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#

# Try to find the v4l2 library.
# Once done this will define the following variables:
#
# libv4l2_FOUND         - System has libv4l2
# libv4l2_INCLUDE_DIRS  - libv4l2 include directory
# libv4l2_LIBRARIES     - libv4l2 libraries
# libv4l2_DEFINITIONS   - Additional compiler flags for libv4l2
# libv4l2_VERSION       - libv4l2 version
# libv4l2_MAJOR_VERSION - libv4l2 major version
# libv4l2_MINOR_VERSION - libv4l2 minor version
# libv4l2_PATCH_VERSION - libv4l2 patch version
# libv4l2_TWEAK_VERSION - libv4l2 tweak version

include(MacroStandardFindModule)
macro_standard_find_module(libv4l2 libv4l2)

# Set package properties if FeatureSummary was included
if(COMMAND set_package_properties)
    set_package_properties(libv4l2 PROPERTIES DESCRIPTION "Video4Linux or V4L is a video capture and output device API and driver framework for the Linux kernel, supporting many USB webcams, TV tuners, and other devices."
                                              URL "https://www.kernel.org/doc/Documentation/video4linux/v4l2-framework.txt")
endif()
