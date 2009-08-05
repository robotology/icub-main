# CMake configuration file roboop source dir
# $Id: CMakeLists.txt,v 1.1 2007/07/24 16:03:09 amaldo Exp $

PROJECT(roboop)
ADD_LIBRARY(roboop
  gnugraph.cpp
  comp_dq.cpp
  comp_dqp.cpp
  delta_t.cpp
  dynamics.cpp
  homogen.cpp
  kinemat.cpp
  robot.cpp
  sensitiv.cpp
  utils.cpp
  quaternion.cpp
  config.cpp
  trajectory.cpp
  clik.cpp
  controller.cpp
  invkine.cpp
  control_select.cpp
  dynamics_sim.cpp
  stewart.cpp)
SET(DEMO_PROGS rtest demo bench demo_2dof_pd)
FOREACH(demo_prog ${DEMO_PROGS})
  ADD_EXECUTABLE(${demo_prog} ${demo_prog}.cpp)
  TARGET_LINK_LIBRARIES(${demo_prog} roboop newmat)
ENDFOREACH(demo_prog)

INSTALL(TARGETS roboop DESTINATION lib)
INSTALL_FILES(/include .h)





