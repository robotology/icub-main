# Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

macro(add_python_unit_test pythonscript)
  if(PYTHON_EXECUTABLE)
    add_test(NAME ${pythonscript} COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/${pythonscript})
    set_tests_properties(${pythonscript} PROPERTIES ENVIRONMENT "PYTHONPATH=$<TARGET_FILE_DIR:${SWIG_MODULE_icub_python_REAL_NAME}>")
  endif()
endmacro()

file(GLOB icub_python_test_files "${CMAKE_CURRENT_SOURCE_DIR}/*.py")

add_python_unit_test(test_load.py)
add_python_unit_test(test_ctrLib.py)
add_python_unit_test(test_iKin.py)
add_python_unit_test(test_skinDynLib.py)
add_python_unit_test(test_optimization.py)

