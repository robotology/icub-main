# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

## Order here is important, libraries go first because they set variables
## that are used by the other modules. 

add_subdirectory(libraries)
add_subdirectory(examples)
add_subdirectory(core)
add_subdirectory(tools)
add_subdirectory(simulators)
add_subdirectory(modules)



