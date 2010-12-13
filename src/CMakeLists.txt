# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

## order here is important, libraries go first because they set variables
## that are used by other modules. This is not a strict requirement but 
## helped solve some sporadice glitches.
add_subdirectory(libraries)
add_subdirectory(examples)
add_subdirectory(core)
add_subdirectory(tools)
#add_subdirectory(simulators)
add_subdirectory(modules)



