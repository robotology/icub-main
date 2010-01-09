
project(libvislab)

cmake_minimum_required(VERSION 2.4)

set(LIBVISLAB_BASE_DIR "${ICUB_DIR}/src/libvislab-snapshot/libvislab" CACHE INTERNAL "")
set(LIBVISLAB_YARP_DIR "${ICUB_DIR}/src/libvislab-snapshot/libvislab_YARP" CACHE INTERNAL "")

add_subdirectory(libvislab)
add_subdirectory(libvislab_YARP)

