# Copyright: (C) 2017 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

add_executable(skinContactTest skinContactTest.cpp)
target_link_libraries(skinContactTest skinDynLib)
add_test(NAME skinContactTest COMMAND skinContactTest)