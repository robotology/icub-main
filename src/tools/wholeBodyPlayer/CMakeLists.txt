# Copyright (C) 2006-2019 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

project(wholeBodyPlayer)

add_executable(${PROJECT_NAME} main.cpp WholeBodyPlayerModule.h WholeBodyPlayerModule.cpp)
target_link_libraries(${PROJECT_NAME} YARP::YARP_os
                                      YARP::YARP_init
                                      YARP::YARP_dev)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

