# Copyright: (C) 2011 IITRBCS
# Authors: Paul Fitzpatrick
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

This is a stub for python/java/... bindings to ICUB libraries.
To build:
 * Build repository as usual.
 * Set ICUB_DIR environment variable to your build directory.
 * Create a new build directory for e.g. python binding, and enter it.
 * Run cmake with the directory this README.txt is in as its source
   directory.
 * Set one of CREATE_PYTHON, CREATE_JAVA, ... options to TRUE/ON.
 * Set *only* *one* option on.  For multiple languages, use multiple
   build directories.
 * Go ahead and compile.  If you have trouble, consider rebuilding
   the ICUB repository with ICUB_SHARED_LIBRARY set to TRUE/ON.
